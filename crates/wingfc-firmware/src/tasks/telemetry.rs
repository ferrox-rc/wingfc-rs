use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Ticker};
use wingfc_core::safety::ArmingStatus;

use crate::drivers::{LedColor, RgbLedDriver};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualStatus {
    pub arming: ArmingStatus,
    pub rc_connected: bool,
    pub rc_packet_tick: bool,
    #[cfg(feature = "gps")]
    pub gps_sats: u8,
    #[cfg(feature = "gps")]
    pub gps_has_fix: bool,
    pub is_calibrating: bool,
    #[cfg(not(feature = "dshot"))]
    pub is_esc_calibrating: bool,
}

impl Default for VisualStatus {
    fn default() -> Self {
        Self {
            arming: ArmingStatus::Disarmed,
            rc_connected: false,
            rc_packet_tick: false,
            #[cfg(feature = "gps")]
            gps_sats: 0,
            #[cfg(feature = "gps")]
            gps_has_fix: false,
            is_calibrating: false,
            #[cfg(not(feature = "dshot"))]
            is_esc_calibrating: false,
        }
    }
}

impl VisualStatus {
    pub const fn calibrating() -> Self {
        Self {
            arming: ArmingStatus::Disarmed,
            rc_connected: false,
            rc_packet_tick: false,
            #[cfg(feature = "gps")]
            gps_sats: 0,
            #[cfg(feature = "gps")]
            gps_has_fix: false,
            is_calibrating: true,
            #[cfg(not(feature = "dshot"))]
            is_esc_calibrating: false,
        }
    }

    #[cfg(not(feature = "dshot"))]
    pub const fn esc_calibrating() -> Self {
        Self {
            arming: ArmingStatus::Disarmed,
            rc_connected: false,
            rc_packet_tick: false,
            #[cfg(feature = "gps")]
            gps_sats: 0,
            #[cfg(feature = "gps")]
            gps_has_fix: false,
            is_calibrating: false,
            is_esc_calibrating: true,
        }
    }

    pub const fn status(arming: ArmingStatus) -> Self {
        Self {
            arming,
            rc_connected: !matches!(arming, ArmingStatus::Failsafe),
            rc_packet_tick: false,
            #[cfg(feature = "gps")]
            gps_sats: 0,
            #[cfg(feature = "gps")]
            gps_has_fix: false,
            is_calibrating: false,
            #[cfg(not(feature = "dshot"))]
            is_esc_calibrating: false,
        }
    }
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
    mut status_rx: Receiver<'static, ThreadModeRawMutex, VisualStatus, 2>,
) {
    defmt::info!("Starting Dual-LED Telemetry task...");

    let mut ticker = Ticker::every(Duration::from_millis(50)); // 20 Hz loop (50ms ticks)
    let mut tick_counter: u32 = 0;
    let mut current_status = VisualStatus::calibrating();

    loop {
        ticker.next().await;
        tick_counter = tick_counter.wrapping_add(1);

        // Fetch latest status if changed
        if let Some(new_status) = status_rx.try_get() {
            current_status = new_status;
        }

        // ==========================================
        // 1. Primary State: RGB LED
        // ==========================================
        #[cfg(not(feature = "dshot"))]
        if current_status.is_esc_calibrating {
            // Fast blink Yellow (100ms on, 100ms off)
            if (tick_counter / 2).is_multiple_of(2) {
                led.set_color(LedColor::Yellow);
            } else {
                led.set_color(LedColor::Off);
            }
        } else if current_status.is_calibrating {
            // Fast blink White (100ms on, 100ms off)
            if (tick_counter / 2).is_multiple_of(2) {
                led.set_color(LedColor::White);
            } else {
                led.set_color(LedColor::Off);
            }
        } else {
            match current_status.arming {
                ArmingStatus::Disarmed => {
                    led.set_color(LedColor::Green);
                }
                ArmingStatus::ArmedAcro => {
                    led.set_color(LedColor::Blue);
                }
                ArmingStatus::ArmedAngle => {
                    led.set_color(LedColor::Cyan);
                }
                #[cfg(feature = "compass")]
                ArmingStatus::ArmedHeadingHold => {
                    // Magenta / Purple (Red + Blue) or solid Blue with blink
                    led.set_color(LedColor::Cyan);
                }
                #[cfg(feature = "baro")]
                ArmingStatus::ArmedAltHold => {
                    led.set_color(LedColor::Green);
                }
                #[cfg(feature = "gps")]
                ArmingStatus::ArmedRth => {
                    // Fast blink Cyan
                    if (tick_counter / 2).is_multiple_of(2) {
                        led.set_color(LedColor::Cyan);
                    } else {
                        led.set_color(LedColor::Off);
                    }
                }
                ArmingStatus::RefusedThrottleRaised | ArmingStatus::RefusedSwitchArmedAtBoot => {
                    let phase = tick_counter % 20; // 1-second cadence
                    if phase == 0 || phase == 1 || phase == 4 || phase == 5 {
                        led.set_color(LedColor::Yellow);
                    } else {
                        led.set_color(LedColor::Off);
                    }
                }
                ArmingStatus::Failsafe => {
                    // Slow blink Red (250ms on, 250ms off)
                    if (tick_counter / 5).is_multiple_of(2) {
                        led.set_color(LedColor::Red);
                    } else {
                        led.set_color(LedColor::Off);
                    }
                }
            }
        }

        #[cfg(feature = "dshot")]
        if current_status.is_calibrating {
            if (tick_counter / 2).is_multiple_of(2) {
                led.set_color(LedColor::White);
            } else {
                led.set_color(LedColor::Off);
            }
        } else {
            match current_status.arming {
                ArmingStatus::Disarmed => {
                    led.set_color(LedColor::Green);
                }
                ArmingStatus::ArmedAcro => {
                    led.set_color(LedColor::Blue);
                }
                ArmingStatus::ArmedAngle => {
                    led.set_color(LedColor::Cyan);
                }
                #[cfg(feature = "compass")]
                ArmingStatus::ArmedHeadingHold => {
                    // Magenta / Purple (Red + Blue) or solid Blue with blink
                    led.set_color(LedColor::Cyan);
                }
                #[cfg(feature = "baro")]
                ArmingStatus::ArmedAltHold => {
                    led.set_color(LedColor::Green);
                }
                #[cfg(feature = "gps")]
                ArmingStatus::ArmedRth => {
                    // Fast blink Cyan
                    if (tick_counter / 2).is_multiple_of(2) {
                        led.set_color(LedColor::Cyan);
                    } else {
                        led.set_color(LedColor::Off);
                    }
                }
                ArmingStatus::RefusedThrottleRaised | ArmingStatus::RefusedSwitchArmedAtBoot => {
                    let phase = tick_counter % 20;
                    if phase == 0 || phase == 1 || phase == 4 || phase == 5 {
                        led.set_color(LedColor::Yellow);
                    } else {
                        led.set_color(LedColor::Off);
                    }
                }
                ArmingStatus::Failsafe => {
                    if (tick_counter / 5).is_multiple_of(2) {
                        led.set_color(LedColor::Red);
                    } else {
                        led.set_color(LedColor::Off);
                    }
                }
            }
        }

        // ==========================================
        // 2. Secondary State: User LED (P0.17)
        // ==========================================
        #[cfg(not(feature = "gps"))]
        {
            // Pattern 1: Heartbeat & RC Link Activity (Non-GPS)
            if !current_status.rc_connected || current_status.arming == ArmingStatus::Failsafe {
                led.set_user_led(false);
            } else if current_status.rc_packet_tick {
                // Micro-toggle / momentary pulse for active packet stream
                led.set_user_led(tick_counter.is_multiple_of(2));
            } else {
                // Link is healthy and connected: Solid ON
                led.set_user_led(true);
            }
        }

        #[cfg(feature = "gps")]
        {
            // Pattern 2: GPS Lock & Navigation Confidence (Sense Plus / GPS builds)
            if !current_status.gps_has_fix || current_status.gps_sats < 4 {
                // Searching for satellites / No 3D Fix: 1 Hz gentle pulse (500ms ON / 500ms OFF)
                let phase = tick_counter % 20;
                led.set_user_led(phase < 10);
            } else if current_status.gps_sats < 8 {
                // 3D Fix acquired, acquiring higher precision: 2 Hz double-flash
                let phase = tick_counter % 10;
                led.set_user_led(phase == 0 || phase == 1 || phase == 3 || phase == 4);
            } else {
                // Full Navigation & Home Lock (>= 8 sats): Solid ON
                led.set_user_led(true);
            }
        }
    }
}

/// Asynchronous CRSF / ExpressLRS telemetry broadcaster.
///
/// Sends Attitude (10Hz), Flight Mode (10Hz), Battery (10Hz), and Navigation/Baro
/// frames out through UARTE0 TX (Pin D6 / P1.11) to the receiver using non-blocking EasyDMA.
#[cfg(feature = "crsf")]
#[embassy_executor::task]
pub async fn crsf_telemetry_task(
    mut tx: embassy_nrf::buffered_uarte::BufferedUarteTx<'static>,
    mut telem_rx: Receiver<'static, ThreadModeRawMutex, FlightTelemetrySnapshot, 2>,
) {
    use embassy_time::{Duration, Ticker};
    use wingfc_core::protocol::crsf::CrsfTelemetryEncoder;

    defmt::info!("Starting CRSF / ELRS Telemetry Task on UARTE0 TX (D6 / P1.11) @ 50 Hz...");
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
                    #[cfg(feature = "compass")]
                    ArmingStatus::ArmedHeadingHold => "HEADFREE",
                    #[cfg(feature = "baro")]
                    ArmingStatus::ArmedAltHold => "ALTHOLD",
                    #[cfg(feature = "gps")]
                    ArmingStatus::ArmedRth => "RTH",
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
