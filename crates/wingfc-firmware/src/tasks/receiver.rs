use embassy_nrf::uarte::UarteRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use wingfc_core::protocol::crsf::CrsfParser;
use wingfc_core::protocol::ibus::IbusParser;
use wingfc_core::protocol::RcChannels;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum ReceiverProtocol {
    Crsf,
    Ibus,
}

#[embassy_executor::task]
pub async fn receiver_task(
    mut rx: UarteRx<'static, embassy_nrf::peripherals::UARTE0>,
    channel_tx: Sender<'static, CriticalSectionRawMutex, RcChannels, 2>,
    protocol: ReceiverProtocol,
) {
    defmt::info!("Starting EasyDMA UART Receiver task ({:?})...", protocol);

    let mut crsf_parser = CrsfParser::new();
    let mut ibus_parser = IbusParser::new();
    let mut byte_buf = [0u8; 1];

    loop {
        // Stream byte-by-byte from EasyDMA so variable-length packets are decoded immediately
        match rx.read(&mut byte_buf).await {
            Ok(()) => {
                let byte = byte_buf[0];
                match protocol {
                    ReceiverProtocol::Crsf => {
                        if let Some(frame) = crsf_parser.parse_byte(byte) {
                            let channels: RcChannels = frame.into();
                            let _ = channel_tx.try_send(channels);
                        }
                    }
                    ReceiverProtocol::Ibus => {
                        if let Some(frame) = ibus_parser.parse_byte(byte) {
                            let channels: RcChannels = frame.into();
                            let _ = channel_tx.try_send(channels);
                        }
                    }
                }
            }
            Err(e) => {
                defmt::warn!("UART read error: {:?}", e);
            }
        }
    }
}
