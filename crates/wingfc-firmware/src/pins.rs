//! Board pinout definitions and hardware mappings for `wingfc-firmware`.
//!
//! This module centralizes physical SoC pin and peripheral assignments, isolating
//! board-specific hardware layouts (such as Seeed Studio XIAO nRF52840 Sense and the
//! future XIAO Sense Plus expansion variant for GPS integration) from application runtime logic.

use embassy_nrf::peripherals;

/// Centralized board pin and peripheral container for the Seeed Studio XIAO nRF52840 Sense.
///
/// Encapsulates all physical GPIOs and on-chip peripheral blocks allocated to flight
/// control functions, motor ESC outputs, servo linkages, IMU buses, and telemetry.
pub struct BoardPins {
    // ==========================================
    // Actuators (PWM)
    // ==========================================
    /// PWM1 peripheral dedicated to Motor 1 ESC (400Hz analog or DShot timing)
    pub pwm_esc: peripherals::PWM1,
    /// Physical Pin: P0.29 (Silkscreen: D3) - Motor 1 ESC signal
    pub pin_motor1: peripherals::P0_29,

    /// PWM0 peripheral dedicated to 50Hz analog servos
    pub pwm_servos: peripherals::PWM0,
    /// Physical Pin: P0.02 (Silkscreen: D0) - Left Elevon (Channel 0)
    pub pin_servo_left: peripherals::P0_02,
    /// Physical Pin: P0.03 (Silkscreen: D1) - Right Elevon (Channel 1)
    pub pin_servo_right: peripherals::P0_03,
    /// Physical Pin: P0.28 (Silkscreen: D2) - Auxiliary Servo / Rudder (Channel 2)
    pub pin_servo_aux: peripherals::P0_28,

    // ==========================================
    // Visual Telemetry (RGB LED)
    // ==========================================
    /// Physical Pin: P0.26 (Internal) - Active-LOW Red Telemetry LED
    pub pin_led_red: peripherals::P0_26,
    /// Physical Pin: P0.30 (Internal) - Active-LOW Green Telemetry LED
    pub pin_led_green: peripherals::P0_30,
    /// Physical Pin: P0.06 (Internal) - Active-LOW Blue Telemetry LED
    pub pin_led_blue: peripherals::P0_06,

    // ==========================================
    // Internal IMU (LSM6DS3TR on TWISPI0)
    // ==========================================
    /// TWISPI0 hardware peripheral for 400kHz Fast-Mode I2C
    pub twispi_imu: peripherals::TWISPI0,
    /// Physical Pin: P0.27 (Internal) - IMU I2C SDA
    pub pin_imu_sda: peripherals::P0_27,
    /// Physical Pin: P0.07 (Internal) - IMU I2C SCL
    pub pin_imu_scl: peripherals::P0_07,
    /// Physical Pin: P0.15 (Internal) - IMU Power Rail Enable (Active HIGH)
    pub pin_imu_pwr: peripherals::P0_15,

    // ==========================================
    // RC Receiver (EasyDMA UARTE0)
    // ==========================================
    /// UARTE0 hardware peripheral for non-blocking EasyDMA serial reception
    pub uarte_rc: peripherals::UARTE0,
    /// Physical Pin: P1.00 (Silkscreen: D6) - RC Receiver RX (CRSF @ 420kBaud / iBus @ 115.2kBaud)
    pub pin_rc_rx: peripherals::P1_00,
}

impl BoardPins {
    /// Extracts and groups pins from the initialized `embassy_nrf::Peripherals`.
    pub fn from_peripherals(p: embassy_nrf::Peripherals) -> Self {
        Self {
            pwm_esc: p.PWM1,
            pin_motor1: p.P0_29,
            pwm_servos: p.PWM0,
            pin_servo_left: p.P0_02,
            pin_servo_right: p.P0_03,
            pin_servo_aux: p.P0_28,
            pin_led_red: p.P0_26,
            pin_led_green: p.P0_30,
            pin_led_blue: p.P0_06,
            twispi_imu: p.TWISPI0,
            pin_imu_sda: p.P0_27,
            pin_imu_scl: p.P0_07,
            pin_imu_pwr: p.P0_15,
            uarte_rc: p.UARTE0,
            pin_rc_rx: p.P1_00,
        }
    }
}

/// Hardware pin reference and future expansion specifications.
///
/// Documents the pin allocation across board revisions, highlighting the migration
/// requirements when upgrading to the Seeed Studio XIAO nRF52840 Sense Plus for GPS.
pub mod pin_reference {
    /// Silkscreen pin labels on the Seeed Studio XIAO nRF52840 form factor.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum XiaoPin {
        D0,
        D1,
        D2,
        D3,
        D4,
        D5,
        D6,
        D7,
        D8,
        D9,
        D10,
    }

    /// Functional allocation of pins in `wingfc-firmware`.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum PinFunction {
        ElevonLeftPwm,
        ElevonRightPwm,
        AuxServoPwm,
        Motor1EscPwm,
        RcReceiverRx,
        ExternalI2cExtSda,
        ExternalI2cExtScl,
        GpsUartRx,
        GpsUartTx,
        BatteryVoltageAdc,
        InternalImuPwr,
        InternalImuI2c,
        RgbStatusLed,
    }

    /// Specification for the future XIAO nRF52840 Sense Plus expansion variant.
    ///
    /// The Sense Plus variant exposes additional pins (D8, D9, D10) and extended solder pads,
    /// enabling the second hardware serial engine (`UARTE1`) to communicate with GPS modules
    /// (NMEA / UBX) while retaining full RC reception (`UARTE0`), multi-channel PWM, and
    /// external I2C magnetometer navigation.
    pub struct GpsExpansionSpec;

    impl GpsExpansionSpec {
        /// Recommended hardware peripheral for GPS communication.
        pub const GPS_UART_PERIPHERAL: &'static str = "UARTE1";
        /// Recommended default baud rate for u-blox M8N/M10Q GPS modules.
        pub const GPS_DEFAULT_BAUD: u32 = 115_200;
        /// Required GPS UART RX pin on the Sense Plus (Silkscreen: D9 / P0.xx).
        pub const GPS_RX_PIN: &'static str = "P0.09 (D9)";
        /// Required GPS UART TX pin on the Sense Plus (Silkscreen: D10 / P0.xx).
        pub const GPS_TX_PIN: &'static str = "P0.10 (D10)";
        /// Required external I2C bus for wing-mounted compass/barometer.
        pub const EXT_I2C_SDA_PIN: &'static str = "P0.04 (D4)";
        pub const EXT_I2C_SCL_PIN: &'static str = "P0.05 (D5)";
    }
}
