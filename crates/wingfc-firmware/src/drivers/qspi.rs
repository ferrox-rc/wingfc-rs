//! QSPI Flash Driver for Puya P25Q16H (2 MB / 16 Mbit NOR Flash).
//!
//! Provides high-speed 32 MHz QSPI page reads, DMA page writes, and 4 KB sector erases.
//! Targets internal NOR flash on the Seeed Studio XIAO nRF52840 Sense.

use core::ops::{Deref, DerefMut};
use embassy_nrf::gpio::Pin;
use embassy_nrf::interrupt::typelevel::Binding;
use embassy_nrf::qspi::{
    AddressMode, Config, Frequency, InterruptHandler, Qspi, ReadOpcode, SpiMode, WriteOpcode,
    WritePageSize,
};
use embassy_nrf::Peri;

/// Puya Semiconductor P25Q16H JEDEC Manufacturer & Device Identification
/// [Manufacturer: 0x85 (Puya), Memory Type: 0x60, Capacity: 0x15 (16 Mbit / 2 MB)]
pub const P25Q16H_JEDEC_ID: [u8; 3] = [0x85, 0x60, 0x15];

/// Total capacity of P25Q16H in bytes (2 MB = 512 sectors * 4 KB)
pub const QSPI_FLASH_SIZE_BYTES: u32 = 2 * 1024 * 1024;
/// Page size in bytes for program operations (256 bytes = 8 Blackbox frames)
pub const QSPI_PAGE_SIZE: usize = 256;
/// Sector size in bytes for erase operations (4 KB = 16 pages)
pub const QSPI_SECTOR_SIZE: usize = 4096;

/// 4-byte aligned 256-byte page buffer required by Nordic EasyDMA hardware.
#[derive(Clone, Copy)]
#[repr(align(4))]
pub struct PageBuffer(pub [u8; QSPI_PAGE_SIZE]);

impl PageBuffer {
    pub const fn new() -> Self {
        Self([0u8; QSPI_PAGE_SIZE])
    }
}

impl Default for PageBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for PageBuffer {
    type Target = [u8; QSPI_PAGE_SIZE];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PageBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum QspiError {
    DeviceNotFound,
    InvalidJedecId([u8; 3]),
    OutOfBounds,
    WriteError,
    ReadError,
    EraseError,
}

pub struct QspiDriver<'d> {
    qspi: Qspi<'d>,
}

#[allow(dead_code)]
impl<'d> QspiDriver<'d> {
    /// Initializes the QSPI bus at 32 MHz in Mode 0 and verifies the Puya P25Q16H JEDEC ID.
    pub async fn init<T: embassy_nrf::qspi::Instance>(
        peri: Peri<'d, T>,
        irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        sck: Peri<'d, impl Pin>,
        csn: Peri<'d, impl Pin>,
        io0: Peri<'d, impl Pin>,
        io1: Peri<'d, impl Pin>,
        io2: Peri<'d, impl Pin>,
        io3: Peri<'d, impl Pin>,
    ) -> Result<Self, QspiError> {
        let mut config = Config::default();
        config.frequency = Frequency::M32;
        config.read_opcode = ReadOpcode::Read4io;
        config.write_opcode = WriteOpcode::Pp4o;
        config.write_page_size = WritePageSize::_256bytes;
        config.spi_mode = SpiMode::Mode0;
        config.address_mode = AddressMode::_24bit;
        config.capacity = QSPI_FLASH_SIZE_BYTES;

        let mut qspi = Qspi::new(peri, irq, sck, csn, io0, io1, io2, io3, config);

        // Verify JEDEC ID (0x9F)
        let mut jedec_id = [0u8; 3];
        qspi.custom_instruction(0x9F, &[], &mut jedec_id)
            .await
            .map_err(|_| QspiError::ReadError)?;

        if jedec_id != P25Q16H_JEDEC_ID {
            defmt::error!(
                "P25Q16H JEDEC ID mismatch: got [0x{:02X}, 0x{:02X}, 0x{:02X}], expected [0x{:02X}, 0x{:02X}, 0x{:02X}]",
                jedec_id[0], jedec_id[1], jedec_id[2],
                P25Q16H_JEDEC_ID[0], P25Q16H_JEDEC_ID[1], P25Q16H_JEDEC_ID[2]
            );
            return Err(QspiError::InvalidJedecId(jedec_id));
        }

        // Check Quad Enable (QE) bit in Status Register 2 (Bit 1, 0x02)
        let mut sr2 = [0u8; 1];
        qspi.custom_instruction(0x35, &[], &mut sr2)
            .await
            .map_err(|_| QspiError::ReadError)?;

        if (sr2[0] & 0x02) == 0 {
            let mut sr1 = [0u8; 1];
            qspi.custom_instruction(0x05, &[], &mut sr1)
                .await
                .map_err(|_| QspiError::ReadError)?;

            // Write Status Register 1 & 2 (0x01) with QE enabled
            qspi.custom_instruction(0x01, &[sr1[0], sr2[0] | 0x02], &mut [])
                .await
                .map_err(|_| QspiError::WriteError)?;
            defmt::info!("P25Q16H Quad Enable (QE) bit latched");
        }

        defmt::info!(
            "Puya P25Q16H 2MB QSPI NOR Flash initialized successfully (32 MHz, Quad I/O)"
        );
        Ok(Self { qspi })
    }

    /// Reads JEDEC ID
    pub async fn read_jedec_id(&mut self) -> Result<[u8; 3], QspiError> {
        let mut id = [0u8; 3];
        self.qspi
            .custom_instruction(0x9F, &[], &mut id)
            .await
            .map_err(|_| QspiError::ReadError)?;
        Ok(id)
    }

    /// Reads a 256-byte page via hardware EasyDMA into an aligned buffer.
    pub async fn read_page(&mut self, address: u32, buf: &mut PageBuffer) -> Result<(), QspiError> {
        self.qspi
            .read(address, &mut buf.0)
            .await
            .map_err(|_| QspiError::ReadError)
    }

    /// Writes a 256-byte page via hardware EasyDMA from an aligned buffer.
    pub async fn write_page(&mut self, address: u32, buf: &PageBuffer) -> Result<(), QspiError> {
        self.qspi
            .write(address, &buf.0)
            .await
            .map_err(|_| QspiError::WriteError)
    }

    /// Erases a 4 KB sector (address must be 4096-byte aligned).
    pub async fn erase_sector(&mut self, address: u32) -> Result<(), QspiError> {
        if address % (QSPI_SECTOR_SIZE as u32) != 0 {
            return Err(QspiError::OutOfBounds);
        }
        self.qspi
            .erase(address)
            .await
            .map_err(|_| QspiError::EraseError)
    }

    /// Reads arbitrary slice from flash (address and length must be 4-byte aligned).
    pub async fn read_aligned(&mut self, address: u32, buf: &mut [u8]) -> Result<(), QspiError> {
        self.qspi
            .read(address, buf)
            .await
            .map_err(|_| QspiError::ReadError)
    }
}
