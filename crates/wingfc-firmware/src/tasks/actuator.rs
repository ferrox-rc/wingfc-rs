//! Actuator Output Task (Tier 3: Independent Hardware Timers).
//!
//! Decoupled from the 208.33 Hz flight loop. Periodically loads whole-frame atomic
//! pulse widths from `ACTUATOR_LATCH` and writes compare registers to hardware PWM blocks.

use embassy_time::{Duration, Ticker};

use crate::bus::{
    ACTUATOR_LATCH, ACT_MOTOR1, ACT_MOTOR2, ACT_SERVO1, ACT_SERVO2, ACT_SERVO3, ACT_SERVO4,
};
#[cfg(feature = "gps")]
use crate::bus::{ACT_SERVO5, ACT_SERVO6, ACT_SERVO7, ACT_SERVO8};
use crate::config::SERVO_PWM_HZ;
#[cfg(not(feature = "dshot"))]
use crate::drivers::EscPwmDriver;
use crate::drivers::ServoPwmDriver;

#[embassy_executor::task]
pub async fn actuator_pwm_task(
    mut servo_pwm: ServoPwmDriver<'static>,
    #[cfg(feature = "gps")]
    mut servo_pwm_ext: ServoPwmDriver<'static>,
    #[cfg(not(feature = "dshot"))]
    mut esc_pwm: EscPwmDriver<'static>,
) {
    defmt::info!("Starting Tier 3 Actuator PWM Task ({} Hz)...", SERVO_PWM_HZ);
    let mut ticker = Ticker::every(Duration::from_hz(SERVO_PWM_HZ as u64));

    loop {
        ticker.next().await;

        // Primary Servos 1..4 (PWM1)
        servo_pwm.set_channel_us(0, ACTUATOR_LATCH.load(ACT_SERVO1));
        servo_pwm.set_channel_us(1, ACTUATOR_LATCH.load(ACT_SERVO2));
        servo_pwm.set_channel_us(2, ACTUATOR_LATCH.load(ACT_SERVO3));
        servo_pwm.set_channel_us(3, ACTUATOR_LATCH.load(ACT_SERVO4));

        // Extended Servos 5..8 (PWM2 on Sense Plus / GPS profile)
        #[cfg(feature = "gps")]
        {
            servo_pwm_ext.set_channel_us(0, ACTUATOR_LATCH.load(ACT_SERVO5));
            servo_pwm_ext.set_channel_us(1, ACTUATOR_LATCH.load(ACT_SERVO6));
            servo_pwm_ext.set_channel_us(2, ACTUATOR_LATCH.load(ACT_SERVO7));
            servo_pwm_ext.set_channel_us(3, ACTUATOR_LATCH.load(ACT_SERVO8));
        }

        // Motors 1 & 2 ESCs (PWM0)
        #[cfg(not(feature = "dshot"))]
        {
            esc_pwm.set_throttle_us(0, ACTUATOR_LATCH.load(ACT_MOTOR1));
            esc_pwm.set_throttle_us(1, ACTUATOR_LATCH.load(ACT_MOTOR2));
        }
    }
}
