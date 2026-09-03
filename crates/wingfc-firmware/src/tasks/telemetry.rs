use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Ticker};
use wingfc_core::safety::ArmingStatus;

use crate::drivers::{LedColor, RgbLedDriver};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VisualStatus {
    Calibration,
    Status(ArmingStatus),
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
            VisualStatus::Calibration => {
                // Fast blink White (100ms on, 100ms off -> toggle every 2 ticks)
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
