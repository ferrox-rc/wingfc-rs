//! Native QSPI Blackbox Flight Logger & Disarmed USB CDC-ACM Offload Task.
//!
//! Captures 50 Hz binary `BlackboxFrame` records during armed flight, buffering
//! 8 frames per 256-byte page into NOR flash via EasyDMA.
//! When disarmed, services USB CDC-ACM commands for log inspection, high-speed dump,
//! and pre-flight sector erasure.

use core::fmt::Write as FmtWrite;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Receiver as WatchReceiver;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{Receiver as CdcReceiver, Sender as CdcSender};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;
use heapless::String;

use wingfc_core::blackbox::BlackboxFrame;
use wingfc_core::safety::ArmingStatus;

use crate::drivers::qspi::{
    PageBuffer, QspiDriver, QSPI_FLASH_SIZE_BYTES, QSPI_PAGE_SIZE, QSPI_SECTOR_SIZE,
};
use crate::tasks::telemetry::VisualStatus;

/// Magic identifier for flash header: "WFCB" (WingFC Blackbox)
const HEADER_MAGIC: [u8; 4] = [0x57, 0x46, 0x43, 0x42];
/// Flash memory offset where binary log ring buffer begins (leaving 16 KB for metadata & cal)
pub const LOG_START_ADDR: u32 = 0x004000;
/// Flash memory offset where binary log ring buffer ends (2 MB)
pub const LOG_END_ADDR: u32 = QSPI_FLASH_SIZE_BYTES;
/// Total log storage capacity in bytes (2,080,768 bytes = 508 sectors)
pub const LOG_CAPACITY_BYTES: u32 = LOG_END_ADDR - LOG_START_ADDR;

/// Fixed-capacity inter-task channel streaming 50 Hz frames from the flight loop
pub static BLACKBOX_CHANNEL: Channel<ThreadModeRawMutex, BlackboxFrame, 16> = Channel::new();

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, HardwareVbusDetect>>) {
    usb.run().await;
}

#[embassy_executor::task]
pub async fn blackbox_task(
    mut qspi: QspiDriver<'static>,
    mut cdc_tx: CdcSender<'static, Driver<'static, HardwareVbusDetect>>,
    mut cdc_rx: CdcReceiver<'static, Driver<'static, HardwareVbusDetect>>,
    mut status_rx: WatchReceiver<'static, ThreadModeRawMutex, VisualStatus, 2>,
) {
    defmt::info!("Starting Blackbox & USB CDC-ACM Offload Task...");

    // 1. Read or initialize Flash Header from Sector 0 (0x000000)
    let mut header_buf = PageBuffer::new();
    let mut write_ptr = LOG_START_ADDR;
    let mut session_count: u32 = 1;

    if qspi.read_page(0x000000, &mut header_buf).await.is_ok() {
        if header_buf[0..4] == HEADER_MAGIC {
            let stored_ptr = u32::from_le_bytes([
                header_buf[4],
                header_buf[5],
                header_buf[6],
                header_buf[7],
            ]);
            let stored_sessions = u32::from_le_bytes([
                header_buf[8],
                header_buf[9],
                header_buf[10],
                header_buf[11],
            ]);
            if (LOG_START_ADDR..=LOG_END_ADDR).contains(&stored_ptr) {
                write_ptr = stored_ptr;
                session_count = stored_sessions.wrapping_add(1);
                defmt::info!(
                    "Recovered Blackbox header: write_ptr=0x{:06X}, sessions={}",
                    write_ptr,
                    session_count
                );
            }
        }
    }

    let mut current_status = ArmingStatus::Disarmed;
    let mut page_buf = PageBuffer::new();
    let mut frames_in_buf: usize = 0;
    let mut usb_cmd_buf = [0u8; 64];
    let mut usb_cmd_len = 0;

    loop {
        // Update arming status from VisualStatus
        if let Some(visual) = status_rx.try_get() {
            let new_status = visual.arming;
            if current_status.is_armed() && !new_status.is_armed() {
                // Transitioned from Armed -> Disarmed: Flush any partial page buffer
                if frames_in_buf > 0 {
                    // Fill remaining page space with 0xFF padding
                    for i in (frames_in_buf * BlackboxFrame::SIZE)..QSPI_PAGE_SIZE {
                        page_buf.0[i] = 0xFF;
                    }
                    if write_ptr < LOG_END_ADDR {
                        let _ = qspi.write_page(write_ptr, &page_buf).await;
                        write_ptr = write_ptr.saturating_add(QSPI_PAGE_SIZE as u32);
                        if write_ptr >= LOG_END_ADDR {
                            write_ptr = LOG_START_ADDR;
                        }
                    }
                    frames_in_buf = 0;
                }
                // Update persistent header with final write pointer
                write_flash_header(&mut qspi, write_ptr, session_count).await;
                defmt::info!(
                    "Blackbox flushed on disarm: write_ptr=0x{:06X}",
                    write_ptr
                );
            }
            current_status = new_status;
        }

        // Drain logging channel
        while let Ok(frame) = BLACKBOX_CHANNEL.try_receive() {
            if current_status.is_armed() {
                let offset = frames_in_buf * BlackboxFrame::SIZE;
                page_buf.0[offset..offset + BlackboxFrame::SIZE].copy_from_slice(&frame.to_bytes());
                frames_in_buf += 1;

                if frames_in_buf == 8 {
                    // Page is full: commit 256 bytes via EasyDMA to QSPI NOR flash
                    if write_ptr < LOG_END_ADDR {
                        let _ = qspi.write_page(write_ptr, &page_buf).await;
                        write_ptr = write_ptr.saturating_add(QSPI_PAGE_SIZE as u32);
                        if write_ptr >= LOG_END_ADDR {
                            write_ptr = LOG_START_ADDR;
                        }
                    }
                    frames_in_buf = 0;
                }
            }
        }

        // Service USB CDC-ACM commands (non-blocking packet polling)
        let mut rx_chunk = [0u8; 64];
        if let Ok(count) = cdc_rx.read_packet(&mut rx_chunk).await {
            for &b in &rx_chunk[..count] {
                if b == b'\n' || b == b'\r' {
                    if usb_cmd_len > 0 {
                        handle_usb_command(
                            &usb_cmd_buf[..usb_cmd_len],
                            &mut qspi,
                            &mut cdc_tx,
                            current_status,
                            &mut write_ptr,
                            session_count,
                        )
                        .await;
                        usb_cmd_len = 0;
                    }
                } else if usb_cmd_len < usb_cmd_buf.len() {
                    usb_cmd_buf[usb_cmd_len] = b;
                    usb_cmd_len += 1;
                }
            }
        }

        Timer::after(Duration::from_millis(5)).await;
    }
}

/// Persists the active write pointer and session counter into Sector 0
async fn write_flash_header(qspi: &mut QspiDriver<'_>, write_ptr: u32, session_count: u32) {
    let _ = qspi.erase_sector(0x000000).await;
    let mut buf = PageBuffer::new();
    buf.0[0..4].copy_from_slice(&HEADER_MAGIC);
    buf.0[4..8].copy_from_slice(&write_ptr.to_le_bytes());
    buf.0[8..12].copy_from_slice(&session_count.to_le_bytes());
    let _ = qspi.write_page(0x000000, &buf).await;
}

/// Processes ASCII commands sent by the companion tool (`scripts/blackbox.py`)
async fn handle_usb_command(
    cmd: &[u8],
    qspi: &mut QspiDriver<'_>,
    cdc_tx: &mut CdcSender<'static, Driver<'static, HardwareVbusDetect>>,
    status: ArmingStatus,
    write_ptr: &mut u32,
    session_count: u32,
) {
    if cmd.starts_with(b"BBL_INFO") {
        let used = if *write_ptr >= LOG_START_ADDR {
            *write_ptr - LOG_START_ADDR
        } else {
            0
        };
        let mut resp: String<128> = String::new();
        let _ = write!(
            resp,
            "OK JEDEC=856015 USED_BYTES={} CAPACITY={} WR_PTR=0x{:06X} SESSIONS={}\r\n",
            used, LOG_CAPACITY_BYTES, *write_ptr, session_count
        );
        let _ = write_all(cdc_tx, resp.as_bytes()).await;
    } else if cmd.starts_with(b"BBL_DUMP") {
        // Safety Interlock: Disallowed while armed
        if status.is_armed() {
            let _ = write_all(cdc_tx, b"ERR:ARMED\r\n").await;
            return;
        }

        let total_bytes = if *write_ptr >= LOG_START_ADDR {
            *write_ptr - LOG_START_ADDR
        } else {
            0
        };

        let mut header: String<64> = String::new();
        let _ = write!(header, "START_DUMP SIZE={}\r\n", total_bytes);
        let _ = write_all(cdc_tx, header.as_bytes()).await;

        let mut addr = LOG_START_ADDR;
        let mut page_buf = PageBuffer::new();
        let mut running_crc = 0u32;

        while addr < *write_ptr {
            if qspi.read_page(addr, &mut page_buf).await.is_ok() {
                running_crc = crc32(running_crc, &page_buf.0);
                if write_all(cdc_tx, &page_buf.0).await.is_err() {
                    break;
                }
            } else {
                break;
            }
            addr = addr.saturating_add(QSPI_PAGE_SIZE as u32);
        }

        let mut footer: String<64> = String::new();
        let _ = write!(footer, "\r\nEND_DUMP CRC32={:08X}\r\n", running_crc);
        let _ = write_all(cdc_tx, footer.as_bytes()).await;
    } else if cmd.starts_with(b"BBL_ERASE") {
        // Safety Interlock: Disallowed while armed
        if status.is_armed() {
            let _ = write_all(cdc_tx, b"ERR:ARMED\r\n").await;
            return;
        }

        let _ = write_all(cdc_tx, b"START_ERASE\r\n").await;

        // Erase 508 sectors across the blackbox region (4 KB each)
        let total_sectors = (LOG_CAPACITY_BYTES / (QSPI_SECTOR_SIZE as u32)) as usize;
        let mut sector_addr = LOG_START_ADDR;

        for i in 0..total_sectors {
            let _ = qspi.erase_sector(sector_addr).await;
            sector_addr += QSPI_SECTOR_SIZE as u32;

            if i % 32 == 0 || i == total_sectors - 1 {
                let mut progress: String<64> = String::new();
                let _ = write!(progress, "ERASING {}/{}\r\n", i + 1, total_sectors);
                let _ = write_all(cdc_tx, progress.as_bytes()).await;
            }
        }

        // Reset write pointer and refresh header
        *write_ptr = LOG_START_ADDR;
        write_flash_header(qspi, *write_ptr, session_count).await;
        let _ = write_all(cdc_tx, b"OK ERASED\r\n").await;
    } else {
        let _ = write_all(cdc_tx, b"ERR:UNKNOWN_CMD\r\n").await;
    }
}

/// Helper to write arbitrary slice data across USB CDC-ACM in 64-byte chunks
async fn write_all<D: embassy_usb::driver::Driver<'static>>(
    sender: &mut CdcSender<'static, D>,
    mut data: &[u8],
) -> Result<(), EndpointError> {
    while !data.is_empty() {
        let chunk = core::cmp::min(data.len(), 64);
        sender.write_packet(&data[..chunk]).await?;
        data = &data[chunk..];
    }
    Ok(())
}

/// Computes standard IEEE 802.3 CRC-32 (polynomial 0xEDB88320)
pub fn crc32(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xEDB88320
            } else {
                c >> 1
            };
        }
    }
    !c
}
