#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::buffered_uarte::{self, BufferedUarte};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::uarte::{self, Baudrate};
use embassy_nrf::wdt::{self, Watchdog};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Duration, Instant};
use static_cell::ConstStaticCell;

use wingfc_core::protocol::RcChannels;

mod bus;
pub mod config;
mod drivers;
pub mod pins;
mod tasks;

#[cfg(not(feature = "dshot"))]
use config::ESC_PWM_HZ;
use config::{NOMINAL_DT, NOMINAL_LOOP_HZ, SERVO_PWM_HZ};

#[cfg(not(feature = "dshot"))]
use drivers::EscPwmDriver;
use drivers::{Lsm6ds3Driver, RgbLedDriver, ServoPwmDriver};
use pins::BoardPins;
#[cfg(feature = "crsf")]
use tasks::{crsf_telemetry_task, FlightTelemetrySnapshot};
#[cfg(any(feature = "baro", feature = "compass"))]
use tasks::{external_sensors_task, ExternalSensorSnapshot};
use tasks::{receiver_task, telemetry_led_task, FlightLoopState, VisualStatus};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    #[cfg(any(feature = "baro", feature = "compass"))]
    TWISPI1 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI1>;
    UARTE0 => buffered_uarte::InterruptHandler<embassy_nrf::peripherals::UARTE0>;
});

static RC_CHANNEL: Channel<ThreadModeRawMutex, RcChannels, 2> = Channel::new();
static STATUS_WATCH: Watch<ThreadModeRawMutex, VisualStatus, 2> = Watch::new();
#[cfg(feature = "crsf")]
static TELEMETRY_WATCH: Watch<ThreadModeRawMutex, FlightTelemetrySnapshot, 2> = Watch::new();
#[cfg(any(feature = "baro", feature = "compass"))]
static EXT_SENSORS_WATCH: Watch<ThreadModeRawMutex, ExternalSensorSnapshot, 2> = Watch::new();

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
    let led_driver = RgbLedDriver::new(
        pins.pin_led_red,
        pins.pin_led_green,
        pins.pin_led_blue,
        pins.pin_user_led,
    );
    spawner.spawn(telemetry_led_task(led_driver, STATUS_WATCH.receiver().unwrap()).unwrap());

    // 2. Initialize Actuators (PWM)
    // Motors 1 & 2 ESC on PWM0 (configurable 50Hz..400Hz analog or DShot)
    #[cfg(not(feature = "dshot"))]
    let esc_config = EscPwmDriver::config(ESC_PWM_HZ);
    #[cfg(not(feature = "dshot"))]
    let pwm_esc_dev =
        SimplePwm::new_2ch(pins.pwm_esc, pins.pin_motor1, pins.pin_motor2, &esc_config);
    #[cfg(not(feature = "dshot"))]
    let mut esc_pwm = EscPwmDriver::new(pwm_esc_dev);

    // Primary Servos (1..4) on PWM1 (50Hz..400Hz)
    let servo_config = ServoPwmDriver::config(SERVO_PWM_HZ);
    let pwm1_servos_dev = SimplePwm::new_4ch(
        pins.pwm1_servos,
        pins.pin_servo1,
        pins.pin_servo2,
        pins.pin_servo3,
        pins.pin_servo4,
        &servo_config,
    );
    let mut servo_pwm = ServoPwmDriver::new(pwm1_servos_dev);

    #[cfg(feature = "gps")]
    let pwm2_servos_dev = SimplePwm::new_4ch(
        pins.pwm2_servos,
        pins.pin_servo5,
        pins.pin_servo6,
        pins.pin_servo7,
        pins.pin_servo8,
        &servo_config,
    );
    #[cfg(feature = "gps")]
    let mut servo_pwm_ext = ServoPwmDriver::new(pwm2_servos_dev);

    // 3. Power on and initialize internal IMU (LSM6DS3TR on TWISPI0)
    let _imu_pwr = Output::new(pins.pin_imu_pwr, Level::High, OutputDrive::HighDrive);

    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    static TWIM_TX_BUF: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);
    let twim = Twim::new(
        pins.twispi_imu,
        Irqs,
        pins.pin_imu_sda,
        pins.pin_imu_scl,
        twim_config,
        TWIM_TX_BUF.take(),
    );
    let imu_int1 = Input::new(pins.pin_imu_int1, Pull::Down);
    let imu_driver = Lsm6ds3Driver::new(twim, imu_int1);

    // 3b. Optional External I2C Sensor Bus (TWISPI1 for Baro & Compass)
    #[cfg(any(feature = "baro", feature = "compass"))]
    {
        let mut twim_ext_config = twim::Config::default();
        twim_ext_config.frequency = twim::Frequency::K400;
        static TWIM_EXT_TX_BUF: ConstStaticCell<[u8; 32]> = ConstStaticCell::new([0; 32]);
        let twim_ext = Twim::new(
            pins.twispi_ext,
            Irqs,
            pins.pin_ext_sda,
            pins.pin_ext_scl,
            twim_ext_config,
            TWIM_EXT_TX_BUF.take(),
        );
        spawner.spawn(external_sensors_task(twim_ext, EXT_SENSORS_WATCH.sender()).unwrap());
    }
    #[cfg(not(any(feature = "baro", feature = "compass")))]
    {
        let _ = (pins.twispi_ext, pins.pin_ext_sda, pins.pin_ext_scl);
    }

    // 4. Configure Buffered EasyDMA UART (UARTE0) matching the selected build-time protocol
    let mut uarte_config = uarte::Config::default();
    #[cfg(feature = "crsf")]
    {
        uarte_config.baudrate = BAUD_420K;
    }
    #[cfg(feature = "ibus")]
    {
        uarte_config.baudrate = Baudrate::Baud115200;
    }
    uarte_config.parity = uarte::Parity::Excluded;

    static RX_BUFFER: ConstStaticCell<[u8; 512]> = ConstStaticCell::new([0; 512]);
    static TX_BUFFER: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);

    let uarte = BufferedUarte::new(
        pins.uarte_rc,
        pins.timer_rc,
        pins.ppi_ch0,
        pins.ppi_ch1,
        pins.ppi_group0,
        pins.pin_rc_rx,
        pins.pin_rc_tx,
        Irqs,
        uarte_config,
        RX_BUFFER.take(),
        TX_BUFFER.take(),
    );
    let (rx, tx) = uarte.split();

    // Spawn Buffered EasyDMA receiver task (channel inputs)
    spawner.spawn(receiver_task(rx, RC_CHANNEL.sender()).unwrap());

    #[cfg(feature = "crsf")]
    {
        // Spawn EasyDMA telemetry task (telemetry downstream to receiver)
        spawner.spawn(crsf_telemetry_task(tx, TELEMETRY_WATCH.receiver().unwrap()).unwrap());
    }
    #[cfg(not(feature = "crsf"))]
    {
        let _ = tx;
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
    #[cfg(any(feature = "baro", feature = "compass"))]
    let mut ext_sensor_receiver = EXT_SENSORS_WATCH.receiver().unwrap();

    // 6. Initialize 2000ms Hardware Watchdog Timer to guard 500Hz loop
    let mut wdt_config = wdt::Config::default();
    wdt_config.timeout_ticks = 2 * 32768; // 2000ms (32768 Hz LFCLK)
    wdt_config.action_during_debug_halt = wdt::HaltConfig::Pause;
    let (_wdt, [mut wdt_handle]) = match Watchdog::try_new(pins.wdt, wdt_config) {
        Ok(res) => res,
        Err(_) => {
            defmt::panic!("Failed to initialize Hardware Watchdog!");
        }
    };

    let max_wait = Duration::from_micros((NOMINAL_DT * 3.0 * 1e6) as u64);
    let mut last_sample_time = Instant::now();
    let mut loop_counter: u32 = 0;
    info!(
        "Starting Hardware DRDY Event-Paced Flight Loop (nominal: {}Hz)...",
        NOMINAL_LOOP_HZ
    );

    loop {
        // 1. Await LSM6DS3 hardware data-ready signal on INT1 (P0.11) via GPIOTE.
        // If the sensor stalls or communication is lost, timeout ensures watchdog and failsafe continue.
        let _ = with_timeout(max_wait, flight_loop.imu.wait_data_ready()).await;
        wdt_handle.pet();
        loop_counter = loop_counter.wrapping_add(1);

        // 2. Measure actual elapsed delta time between samples
        let now = Instant::now();
        let measured_dt = (now - last_sample_time).as_micros() as f32 * 1e-6;
        last_sample_time = now;
        // Bound dt within [0.5 * NOMINAL_DT, 2.0 * NOMINAL_DT] for numerical safety
        let dt = measured_dt.clamp(NOMINAL_DT * 0.5, NOMINAL_DT * 2.0);

        let outputs = flight_loop
            .step(
                &rc_receiver,
                &status_sender,
                #[cfg(any(feature = "baro", feature = "compass"))]
                &mut ext_sensor_receiver,
                dt,
            )
            .await;

        #[cfg(feature = "crsf")]
        {
            // Transmit telemetry at ~50 Hz regardless of loop rate
            const TELEMETRY_DIVIDER: u32 = if NOMINAL_LOOP_HZ >= 100 { NOMINAL_LOOP_HZ / 50 } else { 1 };
            if loop_counter.is_multiple_of(TELEMETRY_DIVIDER) {
                telemetry_sender.send(flight_loop.telemetry_snapshot());
            }
        }

        // Apply physical PWM output updates
        servo_pwm.set_channel_us(0, outputs.servo1_us);
        servo_pwm.set_channel_us(1, outputs.servo2_us);
        servo_pwm.set_channel_us(2, outputs.servo3_us);
        servo_pwm.set_channel_us(3, outputs.servo4_us);
        #[cfg(feature = "gps")]
        {
            servo_pwm_ext.set_channel_us(0, outputs.servo5_us);
            servo_pwm_ext.set_channel_us(1, outputs.servo6_us);
            servo_pwm_ext.set_channel_us(2, outputs.servo7_us);
            servo_pwm_ext.set_channel_us(3, outputs.servo8_us);
        }
        #[cfg(not(feature = "dshot"))]
        {
            esc_pwm.set_throttle_us(0, outputs.motor1_throttle);
            esc_pwm.set_throttle_us(1, outputs.motor2_throttle);
        }
    }
}
