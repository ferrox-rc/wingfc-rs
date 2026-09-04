use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Ticker};
use wingfc_core::safety::ArmingStatus;

use crate::drivers::{LedColor, RgbLedDriver};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VisualStatus {
    #[cfg(not(feature = "dshot"))]
    EscCalibration,
    Calibration,
    Status(ArmingStatus),
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(not(feature = "crsf"), allow(dead_code))]
pub struct FlightTelemetrySnapshot {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub status: ArmingStatus,
    #[cfg(feature = "gps")]
    pub gps_lat: f64,
    #[cfg(feature = "gps")]
    pub gps_lon: f64,
    #[cfg(feature = "gps")]
    pub gps_speed_mps: f32,
    #[cfg(feature = "gps")]
    pub gps_heading_deg: f32,
    #[cfg(feature = "gps")]
    pub gps_alt_m: f32,
    #[cfg(feature = "gps")]
    pub gps_sats: u8,
    #[cfg(feature = "gps")]
    pub gps_has_fix: bool,
    #[cfg(feature = "baro")]
    pub baro_alt_m: f32,
    #[cfg(feature = "baro")]
    pub vario_cms: i16,
}

impl Default for FlightTelemetrySnapshot {
    fn default() -> Self {
        Self {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            status: ArmingStatus::Disarmed,
            #[cfg(feature = "gps")]
            gps_lat: 0.0,
            #[cfg(feature = "gps")]
            gps_lon: 0.0,
            #[cfg(feature = "gps")]
            gps_speed_mps: 0.0,
            #[cfg(feature = "gps")]
            gps_heading_deg: 0.0,
            #[cfg(feature = "gps")]
            gps_alt_m: 0.0,
            #[cfg(feature = "gps")]
            gps_sats: 0,
            #[cfg(feature = "gps")]
            gps_has_fix: false,
            #[cfg(feature = "baro")]
            baro_alt_m: 0.0,
            #[cfg(feature = "baro")]
            vario_cms: 0,
        }
    }
}

#[embassy_executor::task]
pub async fn telemetry_led_task(
    mut led: RgbLedDriver,
    mut status_rx: Receiver<'static, CriticalSectionRawMutex, VisualStatus, 2>,
) {
    defmt::info!("Starting RGB LED Telemetry task...");

    let mut ticker = Ticker::every(Duration::from_millis(50)); // 20 Hz loop
    let mut tick_counter: u32 = 0;
    let mut current_status = VisualStatus::Calibration;

    loop {
        ticker.next().await;
        tick_counter = tick_counter.wrapping_add(1);

        // Fetch latest status if changed
        if let Some(new_status) = status_rx.try_get() {
            current_status = new_status;
        }

        match current_status {
            #[cfg(not(feature = "dshot"))]
            VisualStatus::EscCalibration => {
                // Fast blink Yellow (100ms on, 100ms off): ESC Throttle Calibration Mode
                if (tick_counter / 2) % 2 == 0 {
                    led.set_color(LedColor::Yellow);
                } else {
                    led.set_color(LedColor::Off);
                }
            }
            VisualStatus::Calibration => {
                // Fast blink White (100ms on, 100ms off): Gyro Stationary Calibration
                if (tick_counter / 2) % 2 == 0 {
                    led.set_color(LedColor::White);
                } else {
                    led.set_color(LedColor::Off);
                }
            }
            VisualStatus::Status(ArmingStatus::Disarmed) => {
                // Solid Green: Safe, ready for flight
                led.set_color(LedColor::Green);
            }
            VisualStatus::Status(ArmingStatus::ArmedAcro) => {
                // Solid Blue: Acrobatic Rate Mode
                led.set_color(LedColor::Blue);
            }
            VisualStatus::Status(ArmingStatus::ArmedAngle) => {
                // Solid Cyan: Self-Leveling Stabilized Mode
                led.set_color(LedColor::Cyan);
            }
            VisualStatus::Status(ArmingStatus::RefusedThrottleRaised)
            | VisualStatus::Status(ArmingStatus::RefusedSwitchArmedAtBoot) => {
                // Double blink Yellow: Warning, arming refused
                let phase = tick_counter % 20; // 1-second cycle
                if phase == 0 || phase == 1 || phase == 4 || phase == 5 {
                    led.set_color(LedColor::Yellow);
                } else {
                    led.set_color(LedColor::Off);
                }
            }
            VisualStatus::Status(ArmingStatus::Failsafe) => {
                // Slow blink Red: Failsafe / Loss of RC Link (250ms on, 250ms off)
                if (tick_counter / 5) % 2 == 0 {
                    led.set_color(LedColor::Red);
                } else {
                    led.set_color(LedColor::Off);
                }
            }
        }
    }
}

/// Asynchronous CRSF / ExpressLRS telemetry broadcaster.
///
/// Sends Attitude (10Hz), Flight Mode (10Hz), Battery (10Hz), and Navigation/Baro
/// frames out through UARTE0 TX (Pin D7 / P0.11) to the receiver using non-blocking EasyDMA.
#[cfg(feature = "crsf")]
#[embassy_executor::task]
pub async fn crsf_telemetry_task(
    mut tx: embassy_nrf::uarte::UarteTx<'static, embassy_nrf::peripherals::UARTE0>,
    mut telem_rx: Receiver<'static, CriticalSectionRawMutex, FlightTelemetrySnapshot, 2>,
) {
    use embassy_time::{Duration, Ticker};
    use wingfc_core::protocol::crsf::CrsfTelemetryEncoder;

    defmt::info!("Starting CRSF / ELRS Telemetry Task on UARTE0 TX (D7 / P0.11) @ 50 Hz...");
    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50 Hz slot ticker
    let mut slot: u8 = 0;
    let mut snapshot = FlightTelemetrySnapshot::default();
    let mut buf = [0u8; 32];

    loop {
        ticker.next().await;

        if let Some(new_data) = telem_rx.try_get() {
            snapshot = new_data;
        }

        let len = match slot {
            0 => {
                // Attitude frame (Pitch, Roll, Yaw) - 10 Hz
                CrsfTelemetryEncoder::encode_attitude(
                    snapshot.pitch_rad,
                    snapshot.roll_rad,
                    snapshot.yaw_rad,
                    &mut buf,
                )
            }
            1 => {
                // Flight Mode string - 10 Hz
                let mode_str = match snapshot.status {
                    ArmingStatus::Disarmed => "DISARM",
                    ArmingStatus::ArmedAcro => "ACRO",
                    ArmingStatus::ArmedAngle => "ANGLE",
                    ArmingStatus::Failsafe => "FAILSAFE",
                    ArmingStatus::RefusedThrottleRaised
                    | ArmingStatus::RefusedSwitchArmedAtBoot => "REFUSED",
                };
                CrsfTelemetryEncoder::encode_flight_mode(mode_str, &mut buf)
            }
            2 => {
                // Battery frame - 10 Hz (nominal 3S pack 11.1V = 111 dv, 100% capacity)
                CrsfTelemetryEncoder::encode_battery(111, 0, 0, 100, &mut buf)
            }
            3 => {
                #[cfg(feature = "gps")]
                {
                    if snapshot.gps_has_fix {
                        CrsfTelemetryEncoder::encode_gps(
                            snapshot.gps_lat,
                            snapshot.gps_lon,
                            snapshot.gps_speed_mps,
                            snapshot.gps_heading_deg,
                            snapshot.gps_alt_m,
                            snapshot.gps_sats,
                            &mut buf,
                        )
                    } else {
                        0
                    }
                }
                #[cfg(not(feature = "gps"))]
                {
                    #[cfg(feature = "baro")]
                    {
                        CrsfTelemetryEncoder::encode_baro_altitude(
                            snapshot.baro_alt_m,
                            snapshot.vario_cms,
                            &mut buf,
                        )
                    }
                    #[cfg(not(feature = "baro"))]
                    {
                        0
                    }
                }
            }
            _ => {
                #[cfg(all(feature = "gps", feature = "baro"))]
                {
                    CrsfTelemetryEncoder::encode_baro_altitude(
                        snapshot.baro_alt_m,
                        snapshot.vario_cms,
                        &mut buf,
                    )
                }
                #[cfg(not(all(feature = "gps", feature = "baro")))]
                {
                    0
                }
            }
        };

        slot = (slot + 1) % 5;

        if len > 0 {
            if let Err(e) = tx.write(&buf[..len]).await {
                defmt::warn!("CRSF telemetry TX error: {:?}", e);
            }
        }
    }
}
