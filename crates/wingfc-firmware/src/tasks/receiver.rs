use embassy_nrf::buffered_uarte::BufferedUarteRx;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embedded_io_async::Read;
use wingfc_core::protocol::RcChannels;

#[cfg(all(feature = "crsf", feature = "ibus"))]
compile_error!(
    "Features `crsf` and `ibus` are mutually exclusive. Please enable only one receiver protocol."
);

#[cfg(not(any(feature = "crsf", feature = "ibus")))]
compile_error!("Either feature `crsf` or `ibus` must be enabled.");

#[embassy_executor::task]
pub async fn receiver_task(
    mut rx: BufferedUarteRx<'static>,
    channel_tx: Sender<'static, ThreadModeRawMutex, RcChannels, 2>,
) {
    run_receiver_loop(&mut rx, &channel_tx).await
}

/// Hardware-agnostic RC receiver byte stream parser loop built on [`embedded_io_async::Read`].
pub async fn run_receiver_loop<R: Read>(
    rx: &mut R,
    channel_tx: &Sender<'static, ThreadModeRawMutex, RcChannels, 2>,
) -> ! {
    #[cfg(feature = "crsf")]
    {
        use wingfc_core::protocol::crsf::CrsfParser;
        defmt::info!("DMA UART Receiver: CRSF / ExpressLRS @ 420,000 baud");
        let mut parser = CrsfParser::new();
        let mut chunk = [0u8; 64];

        loop {
            match rx.read(&mut chunk).await {
                Ok(n) => {
                    for &byte in &chunk[..n] {
                        if let Some(frame) = parser.parse_byte(byte) {
                            let channels: RcChannels = frame.into();
                            let _ = channel_tx.try_send(channels);
                        }
                    }
                }
                Err(_) => {
                    defmt::warn!("UART RX read error");
                }
            }
        }
    }

    #[cfg(feature = "ibus")]
    {
        use wingfc_core::protocol::ibus::IbusParser;
        defmt::info!("DMA UART Receiver: FlySky iBus @ 115,200 baud");
        let mut parser = IbusParser::new();
        let mut chunk = [0u8; 64];

        loop {
            match rx.read(&mut chunk).await {
                Ok(n) => {
                    for &byte in &chunk[..n] {
                        if let Some(frame) = parser.parse_byte(byte) {
                            let channels: RcChannels = frame.into();
                            let _ = channel_tx.try_send(channels);
                        }
                    }
                }
                Err(_) => {
                    defmt::warn!("UART RX read error");
                }
            }
        }
    }
}
