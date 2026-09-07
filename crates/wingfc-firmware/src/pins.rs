//! Board pinout definitions and hardware mappings for `wingfc-firmware`.
//!
//! This module centralizes physical SoC pin and peripheral assignments, isolating
//! board-specific hardware layouts (such as Seeed Studio XIAO nRF52840 Sense and the
//! future XIAO Sense Plus expansion variant for GPS integration) from application runtime logic.

use embassy_nrf::peripherals;

/// Centralized board pin and peripheral container for the Seeed Studio XIAO nRF52840 Sense / Sense Plus.
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
    /// Physical Pin: P0.07 (Internal) - IMU I2C SDA
    pub pin_imu_sda: peripherals::P0_07,
    /// Physical Pin: P0.27 (Internal) - IMU I2C SCL
    pub pin_imu_scl: peripherals::P0_27,
    /// Physical Pin: P1.08 (Internal) - IMU Power Rail Enable (Active HIGH)
    pub pin_imu_pwr: peripherals::P1_08,
    /// Physical Pin: P0.11 (Internal) - IMU INT1 (Hardware Data-Ready Interrupt)
    pub pin_imu_int1: peripherals::P0_11,

    // ==========================================
    // RC Receiver (Buffered EasyDMA UARTE0)
    // ==========================================
    /// UARTE0 hardware peripheral for non-blocking EasyDMA serial reception & telemetry
    pub uarte_rc: peripherals::UARTE0,
    /// Hardware TIMER0 dedicated to background DMA double-buffering & timeout detection
    pub timer_rc: peripherals::TIMER0,
    /// Programmable Peripheral Interconnect channel 0
    pub ppi_ch0: peripherals::PPI_CH0,
    /// Programmable Peripheral Interconnect channel 1
    pub ppi_ch1: peripherals::PPI_CH1,
    /// Programmable Peripheral Interconnect Group 0
    pub ppi_group0: peripherals::PPI_GROUP0,
    /// Physical Pin: P1.12 (Silkscreen: D7) - RC Receiver RX (from receiver TX)
    pub pin_rc_rx: peripherals::P1_12,
    /// Physical Pin: P1.11 (Silkscreen: D6) - RC Telemetry TX (to receiver RX)
    pub pin_rc_tx: peripherals::P1_11,

    // ==========================================
    // GPS Receiver (EasyDMA UARTE1 - XIAO Sense Plus)
    // ==========================================
    #[cfg(feature = "gps")]
    /// UARTE1 hardware peripheral for GPS communication (Sense Plus variant)
    pub uarte_gps: peripherals::UARTE1,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.14 (Silkscreen: D9) - GPS UART RX
    pub pin_gps_rx: peripherals::P1_14,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.15 (Silkscreen: D10) - GPS UART TX
    pub pin_gps_tx: peripherals::P1_15,
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
            pin_gps_rx: p.P1_14,
            #[cfg(feature = "gps")]
            pin_gps_tx: p.P1_15,
        }
    }
}
