#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use wingfc_core::{MadgwickAHRS, PidController, PidConfig, AirframeMixer, AirframeType, Vector3};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("=========================================");
    info!(" WingFC-rs: Next-Gen Flight Controller   ");
    info!(" Architecture: Cortex-M4F (nRF52840)     ");
    info!("=========================================");

    let _p = embassy_nrf::init(Default::default());

    let mut ahrs = MadgwickAHRS::new(0.04);
    let mut roll_rate_pid = PidController::new(PidConfig {
        kp: 0.15,
        ki: 0.05,
        kd: 0.008,
        max_integral: 0.4,
        dterm_lpf_hz: 40.0,
    }, 0.002);

    let mixer = AirframeMixer::new(AirframeType::Elevon);

    // 500 Hz control loop (2ms)
    let mut ticker = Ticker::every(Duration::from_hz(500));
    info!("Starting 500Hz Flight Control Loop...");

    loop {
        ticker.next().await;

        // Mock IMU readings for initial bring-up
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);

        ahrs.update_6dof(gyro, accel, 0.002);
        let (pitch_rad, roll_rad, yaw_rad) = ahrs.get_euler_angles();

        let _roll_mix = roll_rate_pid.update(0.0, gyro.x, 0.002);
        let outputs = mixer.mix(pitch_rad, roll_rad, yaw_rad, 1000);

        defmt::trace!("S1: {} us, S2: {} us", outputs.servo1_us, outputs.servo2_us);
    }
}
