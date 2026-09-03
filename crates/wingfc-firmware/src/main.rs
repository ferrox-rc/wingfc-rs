#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::uarte::{self, Baudrate, UarteRx};
use embassy_nrf::bind_interrupts;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

use wingfc_core::protocol::RcChannels;

mod drivers;
mod tasks;

use drivers::{EscPwmDriver, Lsm6ds3Driver, RgbLedDriver, ServoPwmDriver};
use tasks::{FlightLoopState, receiver_task, telemetry_led_task, VisualStatus};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    UARTE0 => uarte::InterruptHandler<embassy_nrf::peripherals::UARTE0>;
});

static RC_CHANNEL: Channel<CriticalSectionRawMutex, RcChannels, 2> = Channel::new();
static STATUS_WATCH: Watch<CriticalSectionRawMutex, VisualStatus, 2> = Watch::new();

#[cfg(feature = "crsf")]
// CRSF standard baud rate is 420,000 baud: (420000 * 2^32) / 16000000 = 0x06BA4000
const BAUD_420K: Baudrate = Baudrate(0x06BA4000);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("=========================================");
    info!(" WingFC-rs: Next-Gen Flight Controller   ");
    info!(" Architecture: Cortex-M4F (nRF52840)     ");
    #[cfg(feature = "crsf")]
    info!(" Receiver Protocol: CRSF / ExpressLRS    ");
    #[cfg(feature = "ibus")]
    info!(" Receiver Protocol: FlySky iBus          ");
    info!("=========================================");

    let p = embassy_nrf::init(Default::default());

    // 1. Initialize Tri-Color Visual Telemetry (RGB LED) on Seeed XIAO
    let led_driver = RgbLedDriver::new(p.P0_26, p.P0_30, p.P0_06);
    let status_sender = STATUS_WATCH.sender();
    let status_receiver = STATUS_WATCH.receiver().unwrap();

    spawner.spawn(telemetry_led_task(led_driver, status_receiver)).unwrap();

    // Enable IMU Power Rail on Seeed XIAO Sense (P0.15 set HIGH)
    let _imu_power = Output::new(p.P0_15, Level::High, OutputDrive::Standard);

    // 2. Configure I2C (TWISPI0) for LSM6DS3TR IMU (400 kHz)
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    // XIAO Sense internal IMU wiring: SCL = P0.07, SDA = P0.27
    let twim = Twim::new(p.TWISPI0, Irqs, p.P0_27, p.P0_07, twim_config);
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
    let rx = UarteRx::new(p.UARTE0, Irqs, p.P1_00, uarte_config);

    // Spawn EasyDMA receiver task
    spawner
        .spawn(receiver_task(rx, RC_CHANNEL.sender()))
        .unwrap();

    // 4. Configure Multi-Frequency PWM Drivers:
    // Instance 0 (PWM0): 50Hz for Servos (20ms frame)
    let servo_pwm_hw = SimplePwm::new_3ch(
        p.PWM0,
        p.P0_02, // D0 (Servo 1: Left Elevon)
        p.P0_03, // D1 (Servo 2: Right Elevon)
        p.P0_28, // D2 (Servo 4: Aux)
    );
    let mut servo_pwm = ServoPwmDriver::new(servo_pwm_hw);

    // Instance 1 (PWM1): 400Hz High-Speed PWM for Motor ESC (2.5ms frame - 8x faster response)
    let esc_pwm_hw = SimplePwm::new_1ch(
        p.PWM1,
        p.P0_29, // D3 (Motor 1: ESC)
    );
    let mut esc_pwm = EscPwmDriver::new(esc_pwm_hw);

    // 5. Initialize Flight Loop & Pre-Arm Safety State Machine
    let mut flight_loop = FlightLoopState::new(imu_driver);
    flight_loop.init(&status_sender).await;

    let mut ticker = Ticker::every(Duration::from_hz(500)); // 500 Hz hard real-time loop (2ms)
    info!("Starting 500Hz Real-Time Flight Loop...");

    let rc_receiver = RC_CHANNEL.receiver();

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
