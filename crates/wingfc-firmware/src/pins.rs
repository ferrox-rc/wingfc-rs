//! Board pinout definitions and hardware mappings for `wingfc-firmware`.
//!
//! This module centralizes physical SoC pin and peripheral assignments, isolating
//! board-specific hardware layouts (such as Seeed Studio XIAO nRF52840 Sense and the
//! XIAO Sense Plus expansion variant) from application runtime logic.

use embassy_nrf::peripherals;
use embassy_nrf::Peri;

/// Centralized board pin and peripheral container for the Seeed Studio XIAO nRF52840 Sense / Sense Plus.
///
/// Encapsulates all physical GPIOs and on-chip peripheral blocks allocated to flight
/// control functions, motor ESC outputs, servo linkages, IMU buses, and telemetry.
pub struct BoardPins {
    /// Hardware Watchdog Timer
    pub wdt: Peri<'static, peripherals::WDT>,

    // ==========================================
    // Actuators (PWM0: ESC Motors 1 & 2)
    // ==========================================
    /// PWM0 peripheral dedicated to Motor 1 & Motor 2 ESCs (50Hz..400Hz analog or DShot timing)
    pub pwm_esc: Peri<'static, peripherals::PWM0>,
    /// Physical Pin: P0.29 (Silkscreen: D3) - Motor 1 ESC signal
    pub pin_motor1: Peri<'static, peripherals::P0_29>,
    /// Physical Pin: P0.04 (Silkscreen: D4) - Motor 2 ESC signal
    pub pin_motor2: Peri<'static, peripherals::P0_04>,

    // ==========================================
    // Actuators (PWM1: Primary Servos 1, 2, 4, 5)
    // ==========================================
    /// PWM1 peripheral dedicated to primary 4 servo outputs (50Hz..400Hz)
    pub pwm1_servos: Peri<'static, peripherals::PWM1>,
    /// Physical Pin: P0.02 (Silkscreen: D0) - Servo 1 (Left Elevon / Primary Aileron)
    pub pin_servo1: Peri<'static, peripherals::P0_02>,
    /// Physical Pin: P0.03 (Silkscreen: D1) - Servo 2 (Right Elevon / Elevator / Right V-Tail)
    pub pin_servo2: Peri<'static, peripherals::P0_03>,
    /// Physical Pin: P0.28 (Silkscreen: D2) - Servo 4 (Rudder / Left V-Tail / Aux)
    pub pin_servo4: Peri<'static, peripherals::P0_28>,
    /// Physical Pin: P0.05 (Silkscreen: D5) - Servo 5 (Secondary Aileron / Flap / Aux)
    pub pin_servo5: Peri<'static, peripherals::P0_05>,

    // ==========================================
    // Actuators (PWM2: Secondary Servos 6..9 - Sense Plus Only)
    // ==========================================
    #[cfg(feature = "gps")]
    /// PWM2 peripheral dedicated to secondary 4 servo outputs (Sense Plus variant only)
    pub pwm2_servos: Peri<'static, peripherals::PWM2>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.13 (Silkscreen: D8) - Servo 6 (Secondary Elevator / Aux)
    pub pin_servo6: Peri<'static, peripherals::P1_13>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.14 (Silkscreen: D9) - Servo 7 (Aux)
    pub pin_servo7: Peri<'static, peripherals::P1_14>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.15 (Silkscreen: D10) - Servo 8 (Aux)
    pub pin_servo8: Peri<'static, peripherals::P1_15>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P0.15 (Silkscreen: D11) - Servo 9 (Aux)
    pub pin_servo9: Peri<'static, peripherals::P0_15>,

    // ==========================================
    // Visual Telemetry (RGB LED)
    // ==========================================
    /// Physical Pin: P0.26 (Internal) - Active-LOW Red Telemetry LED
    pub pin_led_red: Peri<'static, peripherals::P0_26>,
    /// Physical Pin: P0.30 (Internal) - Active-LOW Green Telemetry LED
    pub pin_led_green: Peri<'static, peripherals::P0_30>,
    /// Physical Pin: P0.06 (Internal) - Active-LOW Blue Telemetry LED
    pub pin_led_blue: Peri<'static, peripherals::P0_06>,
    /// Physical Pin: P0.17 (Internal) - Active-LOW Red Charge / Discrete User Telemetry LED
    pub pin_user_led: Peri<'static, peripherals::P0_17>,

    // ==========================================
    // Internal IMU (LSM6DS3TR on TWISPI0)
    // ==========================================
    /// TWISPI0 hardware peripheral for 400kHz Fast-Mode I2C
    pub twispi_imu: Peri<'static, peripherals::TWISPI0>,
    /// Physical Pin: P0.07 (Internal) - IMU I2C SDA
    pub pin_imu_sda: Peri<'static, peripherals::P0_07>,
    /// Physical Pin: P0.27 (Internal) - IMU I2C SCL
    pub pin_imu_scl: Peri<'static, peripherals::P0_27>,
    /// Physical Pin: P1.08 (Internal) - IMU Power Rail Enable (Active HIGH)
    pub pin_imu_pwr: Peri<'static, peripherals::P1_08>,
    /// Physical Pin: P0.11 (Internal) - IMU INT1 (Hardware Data-Ready Interrupt)
    pub pin_imu_int1: Peri<'static, peripherals::P0_11>,

    // ==========================================
    // RC Receiver (Buffered EasyDMA UARTE0)
    // ==========================================
    /// UARTE0 hardware peripheral for non-blocking EasyDMA serial reception & telemetry
    pub uarte_rc: Peri<'static, peripherals::UARTE0>,
    /// Hardware TIMER0 dedicated to background DMA double-buffering & timeout detection
    pub timer_rc: Peri<'static, peripherals::TIMER0>,
    /// Programmable Peripheral Interconnect channel 0
    pub ppi_ch0: Peri<'static, peripherals::PPI_CH0>,
    /// Programmable Peripheral Interconnect channel 1
    pub ppi_ch1: Peri<'static, peripherals::PPI_CH1>,
    /// Programmable Peripheral Interconnect Group 0
    pub ppi_group0: Peri<'static, peripherals::PPI_GROUP0>,
    /// Physical Pin: P1.12 (Silkscreen: D7) - RC Receiver RX (from receiver TX)
    pub pin_rc_rx: Peri<'static, peripherals::P1_12>,
    /// Physical Pin: P1.11 (Silkscreen: D6) - RC Telemetry TX (to receiver RX)
    pub pin_rc_tx: Peri<'static, peripherals::P1_11>,

    // ==========================================
    // GPS Receiver (EasyDMA UARTE1 - XIAO Sense Plus)
    // ==========================================
    #[cfg(feature = "gps")]
    /// UARTE1 hardware peripheral for GPS communication (Sense Plus variant)
    pub uarte_gps: Peri<'static, peripherals::UARTE1>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P0.09 (Silkscreen: D14 / NFC1) - GPS UART RX
    pub pin_gps_rx: Peri<'static, peripherals::P0_09>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P0.10 (Silkscreen: D15 / NFC2) - GPS UART TX
    pub pin_gps_tx: Peri<'static, peripherals::P0_10>,
}

impl BoardPins {
    /// Extracts and groups pins from the initialized `embassy_nrf::Peripherals`.
    pub fn from_peripherals(p: embassy_nrf::Peripherals) -> Self {
        Self {
            wdt: p.WDT,
            pwm_esc: p.PWM0,
            pin_motor1: p.P0_29,
            pin_motor2: p.P0_04,
            pwm1_servos: p.PWM1,
            pin_servo1: p.P0_02,
            pin_servo2: p.P0_03,
            pin_servo4: p.P0_28,
            pin_servo5: p.P0_05,
            #[cfg(feature = "gps")]
            pwm2_servos: p.PWM2,
            #[cfg(feature = "gps")]
            pin_servo6: p.P1_13,
            #[cfg(feature = "gps")]
            pin_servo7: p.P1_14,
            #[cfg(feature = "gps")]
            pin_servo8: p.P1_15,
            #[cfg(feature = "gps")]
            pin_servo9: p.P0_15,
            pin_led_red: p.P0_26,
            pin_led_green: p.P0_30,
            pin_led_blue: p.P0_06,
            pin_user_led: p.P0_17,
            twispi_imu: p.TWISPI0,
            pin_imu_sda: p.P0_07,
            pin_imu_scl: p.P0_27,
            pin_imu_pwr: p.P1_08,
            pin_imu_int1: p.P0_11,
            uarte_rc: p.UARTE0,
            timer_rc: p.TIMER0,
            ppi_ch0: p.PPI_CH0,
            ppi_ch1: p.PPI_CH1,
            ppi_group0: p.PPI_GROUP0,
            pin_rc_rx: p.P1_12,
            pin_rc_tx: p.P1_11,
            #[cfg(feature = "gps")]
            uarte_gps: p.UARTE1,
            #[cfg(feature = "gps")]
            pin_gps_rx: p.P0_09,
            #[cfg(feature = "gps")]
            pin_gps_tx: p.P0_10,
        }
    }
}
