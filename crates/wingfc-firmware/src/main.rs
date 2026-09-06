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
#[cfg(feature = "crsf")]
use embassy_nrf::uarte::Uarte;
#[cfg(feature = "ibus")]
use embassy_nrf::uarte::UarteRx;
use embassy_nrf::uarte::{self, Baudrate};
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
#[cfg(feature = "crsf")]
use tasks::{crsf_telemetry_task, FlightTelemetrySnapshot};
use tasks::{receiver_task, telemetry_led_task, FlightLoopState, VisualStatus};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    UARTE0 => uarte::InterruptHandler<embassy_nrf::peripherals::UARTE0>;
});

static RC_CHANNEL: Channel<CriticalSectionRawMutex, RcChannels, 2> = Channel::new();
static STATUS_WATCH: Watch<CriticalSectionRawMutex, VisualStatus, 2> = Watch::new();
#[cfg(feature = "crsf")]
static TELEMETRY_WATCH: Watch<CriticalSectionRawMutex, FlightTelemetrySnapshot, 2> = Watch::new();

#[cfg(all(feature = "crsf", feature = "ibus"))]
compile_error!("Features 'crsf' and 'ibus' are mutually exclusive. Exactly one receiver protocol must be selected.");

#[cfg(not(any(feature = "crsf", feature = "ibus")))]
compile_error!("A receiver protocol feature must be enabled: select either 'crsf' or 'ibus'.");

#[cfg(feature = "crsf")]
const BAUD_420K: Baudrate = Baudrate::from_bits(0x06B50000u32);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let pins = BoardPins::from_peripherals(p);

    info!("Initializing WingFC-rs bare-metal flight controller...");

    // 1. Initialize Visual Telemetry (RGB LED)
    let led_driver = RgbLedDriver::new(pins.pin_led_red, pins.pin_led_green, pins.pin_led_blue);
    spawner
        .spawn(telemetry_led_task(
            led_driver,
            STATUS_WATCH.receiver().unwrap(),
        ))
        .unwrap();

    // 2. Initialize Actuators (PWM)
    // Motor 1 ESC on PWM1 (configurable 50Hz..400Hz analog or DShot)
    let pwm_esc_dev = SimplePwm::new_1ch(pins.pwm_esc, pins.pin_motor1);
    let mut esc_pwm = EscPwmDriver::new(pwm_esc_dev, ESC_PWM_HZ);

    // Servos (Elevons + Aux) on PWM0 (50Hz analog)
    let pwm_servos_dev = SimplePwm::new_3ch(
        pins.pwm_servos,
        pins.pin_servo_left,
        pins.pin_servo_right,
        pins.pin_servo_aux,
    );
    let mut servo_pwm = ServoPwmDriver::new(pwm_servos_dev, SERVO_PWM_HZ);

    // 3. Power on and initialize internal IMU (LSM6DS3TR on TWISPI0)
    let mut _pwr = Output::new(pins.pin_imu_pwr, Level::High, OutputDrive::Standard);

    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    let twim = Twim::new(
        pins.twispi_imu,
        Irqs,
        pins.pin_imu_sda,
        pins.pin_imu_scl,
        twim_config,
    );
    let imu_driver = Lsm6ds3Driver::new(twim);

    // 4. Configure EasyDMA UART (UARTE0) matching the selected build-time protocol
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

    #[cfg(feature = "crsf")]
    {
        let uarte = Uarte::new(
            pins.uarte_rc,
            Irqs,
            pins.pin_rc_rx,
            pins.pin_rc_tx,
            uarte_config,
        );
        let (tx, rx) = uarte.split();

        // Spawn EasyDMA receiver task (channel inputs)
        spawner
            .spawn(receiver_task(rx, RC_CHANNEL.sender()))
            .unwrap();

        // Spawn EasyDMA telemetry task (telemetry downstream to receiver)
        spawner
            .spawn(crsf_telemetry_task(tx, TELEMETRY_WATCH.receiver().unwrap()))
            .unwrap();
    }

    #[cfg(feature = "ibus")]
    {
        let rx = UarteRx::new(pins.uarte_rc, Irqs, pins.pin_rc_rx, uarte_config);

        // Spawn EasyDMA receiver task
        spawner
            .spawn(receiver_task(rx, RC_CHANNEL.sender()))
            .unwrap();
    }

    // 5. Initialize Flight Loop and calibrate Gyro
    // (ESC throttle range calibration is only checked if PWM is used; unnecessary if DShot is active)
    let status_sender = STATUS_WATCH.sender();
    let rc_receiver = RC_CHANNEL.receiver();
    let mut flight_loop = FlightLoopState::new(imu_driver);
    #[cfg(not(feature = "dshot"))]
    flight_loop
        .init(&rc_receiver, &status_sender, &mut esc_pwm)
        .await;
    #[cfg(feature = "dshot")]
    flight_loop.init(&status_sender).await;

    #[cfg(feature = "crsf")]
    let telemetry_sender = TELEMETRY_WATCH.sender();

    let loop_hz = (1.0 / DT) as u64;
    let mut ticker = Ticker::every(Duration::from_hz(loop_hz)); // 500 Hz hard real-time loop (2ms)
    let mut loop_counter: u32 = 0;
    info!("Starting 500Hz Real-Time Flight Loop...");

    loop {
        ticker.next().await;
        loop_counter = loop_counter.wrapping_add(1);

        let outputs = flight_loop.step(&rc_receiver, &status_sender).await;

        #[cfg(feature = "crsf")]
        {
            if loop_counter.is_multiple_of(10) {
                telemetry_sender.send(flight_loop.telemetry_snapshot());
            }
        }

        // Apply physical PWM output updates
        // Servos operate at 50 Hz; update their pulse widths every 10 loop ticks (20 ms)
        if loop_counter.is_multiple_of(10) {
            servo_pwm.set_channel_us(0, outputs.servo1_us);
            servo_pwm.set_channel_us(1, outputs.servo2_us);
            servo_pwm.set_channel_us(2, outputs.servo4_us);
        }
        esc_pwm.set_throttle_us(outputs.motor1_throttle);
    }
}
