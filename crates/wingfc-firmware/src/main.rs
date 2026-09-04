#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::uarte::{self, Baudrate, UarteRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

use wingfc_core::protocol::RcChannels;

pub mod config;
mod drivers;
pub mod pins;
mod tasks;

use config::{DT, ESC_PWM_HZ, SERVO_PWM_HZ};
use drivers::{EscPwmDriver, Lsm6ds3Driver, RgbLedDriver, ServoPwmDriver};
use pins::BoardPins;
use tasks::{receiver_task, telemetry_led_task, FlightLoopState, VisualStatus};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    UARTE0 => uarte::InterruptHandler<embassy_nrf::peripherals::UARTE0>;
});

static RC_CHANNEL: Channel<CriticalSectionRawMutex, RcChannels, 2> = Channel::new();
static STATUS_WATCH: Watch<CriticalSectionRawMutex, VisualStatus, 2> = Watch::new();

#[cfg(all(feature = "crsf", feature = "ibus"))]
compile_error!("Features 'crsf' and 'ibus' are mutually exclusive. Exactly one receiver protocol must be selected.");

#[cfg(not(any(feature = "crsf", feature = "ibus")))]
compile_error!("A receiver protocol feature must be enabled: select either 'crsf' or 'ibus'.");

#[cfg(feature = "crsf")]
// CRSF standard baud rate is 420,000 baud: (420000 * 2^32) / 16000000 = 0x06BA4000
const BAUD_420K: Baudrate = Baudrate(0x06BA4000);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("=========================================");
    info!(" WingFC-rs: Next-Gen Flight Controller   ");
    info!(" Architecture: Cortex-M4F (nRF52840)     ");
    #[cfg(feature = "crsf")]
    info!(" RX Protocol: CRSF / ELRS @ 420 kBaud    ");
    #[cfg(feature = "ibus")]
    info!(" RX Protocol: FlySky iBus @ 115.2 kBaud  ");
    #[cfg(feature = "dshot")]
    info!(" Motor Output: Digital DShot             ");
    #[cfg(not(feature = "dshot"))]
    info!(" Motor Output: Analog PWM ({} Hz)        ", ESC_PWM_HZ);
    info!(" Servo Output: 3-ch PWM ({} Hz)          ", SERVO_PWM_HZ);
    #[cfg(feature = "nav")]
    info!(" Nav Suite: Baro (KF) + 9-DoF Compass    ");
    #[cfg(all(feature = "baro", not(feature = "nav")))]
    info!(" Sensor: Barometer enabled               ");
    #[cfg(all(feature = "compass", not(feature = "nav")))]
    info!(" Sensor: Magnetometer/Compass enabled    ");
    info!("=========================================");

    let p = embassy_nrf::init(Default::default());
    let pins = BoardPins::from_peripherals(p);

    // =========================================================================
    // STEP 0: Universal ESC Startup Rule #1 & #2: IMMEDIATE LOW SIGNAL ON BOOT
    // Initialize hardware PWM before any I2C/IMU/UART processing begins so the
    // ESC immediately sees a clean 1000us pulse and begins its arming countdown.
    // =========================================================================
    let esc_pwm_hw = SimplePwm::new_1ch(
        pins.pwm_esc,
        pins.pin_motor1, // D3 (Motor 1: ESC) - configurable frequency PWM
    );
    let mut esc_pwm = EscPwmDriver::new(esc_pwm_hw, ESC_PWM_HZ);

    let servo_pwm_hw = SimplePwm::new_3ch(
        pins.pwm_servos,
        pins.pin_servo_left,  // D0 (Servo 1: Left Elevon)
        pins.pin_servo_right, // D1 (Servo 2: Right Elevon)
        pins.pin_servo_aux,   // D2 (Servo 4: Aux)
    );
    let mut servo_pwm = ServoPwmDriver::new(servo_pwm_hw, SERVO_PWM_HZ);

    // 1. Initialize Tri-Color Visual Telemetry (RGB LED) on Seeed XIAO
    let led_driver = RgbLedDriver::new(pins.pin_led_red, pins.pin_led_green, pins.pin_led_blue);
    let status_sender = STATUS_WATCH.sender();
    let status_receiver = STATUS_WATCH.receiver().unwrap();

    spawner
        .spawn(telemetry_led_task(led_driver, status_receiver))
        .unwrap();

    // Enable IMU Power Rail on Seeed XIAO Sense (P0.15 set HIGH)
    let _imu_power = Output::new(pins.pin_imu_pwr, Level::High, OutputDrive::Standard);

    // 2. Configure I2C (TWISPI0) for LSM6DS3TR IMU (400 kHz)
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    // XIAO Sense internal IMU wiring: SCL = P0.07, SDA = P0.27
    let twim = Twim::new(
        pins.twispi_imu,
        Irqs,
        pins.pin_imu_sda,
        pins.pin_imu_scl,
        twim_config,
    );
    let imu_driver = Lsm6ds3Driver::new(twim);

    // 3. Configure EasyDMA UART (UARTE0) matching the selected build-time protocol
    let mut uarte_config = uarte::Config::default();
    #[cfg(feature = "crsf")]
    {
        uarte_config.baudrate = BAUD_420K;
    }
    #[cfg(feature = "ibus")]
    {
        uarte_config.baudrate = Baudrate::BAUD115200;
    }
    uarte_config.parity = uarte::Parity::EXCLUDED;
    let rx = UarteRx::new(pins.uarte_rc, Irqs, pins.pin_rc_rx, uarte_config);

    // Spawn EasyDMA receiver task
    spawner
        .spawn(receiver_task(rx, RC_CHANNEL.sender()))
        .unwrap();

    // 4. Initialize Flight Loop and calibrate Gyro
    // (ESC throttle range calibration is only checked if PWM is used; unnecessary if DShot is active)
    let rc_receiver = RC_CHANNEL.receiver();
    let mut flight_loop = FlightLoopState::new(imu_driver);
    #[cfg(not(feature = "dshot"))]
    flight_loop
        .init(&rc_receiver, &status_sender, &mut esc_pwm)
        .await;
    #[cfg(feature = "dshot")]
    flight_loop.init(&status_sender).await;

    let loop_hz = (1.0 / DT) as u64;
    let mut ticker = Ticker::every(Duration::from_hz(loop_hz)); // 500 Hz hard real-time loop (2ms)
    info!("Starting 500Hz Real-Time Flight Loop...");

    loop {
        ticker.next().await;

        let outputs = flight_loop.step(&rc_receiver, &status_sender).await;

        // Apply physical PWM output updates via EasyDMA
        servo_pwm.set_channel_us(0, outputs.servo1_us);
        servo_pwm.set_channel_us(1, outputs.servo2_us);
        servo_pwm.set_channel_us(2, outputs.servo4_us);
        esc_pwm.set_throttle_us(outputs.motor1_throttle);
    }
}
