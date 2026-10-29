//! Board pinout definitions and hardware mappings for `wingfc-firmware`.
//!
//! This module centralizes physical SoC pin and peripheral assignments, isolating
//! board-specific hardware layouts (such as Seeed Studio XIAO nRF52840 Sense and the
//! XIAO Sense Plus expansion variant) from application runtime logic.
//!
//! ### Physical Castellated Pad Layout
//!
//! Only physical castellated edge pads are used to ensure direct surface-mount
//! compatibility with carrier PCBs. Underside test pads are strictly avoided.
//!
//! #### Standard XIAO Sense (14 Castellated Pads: D0..D10, 5V, GND, 3V3):
//! - **D0..D2 (`P0.02`, `P0.03`, `P0.28`)**: Servos 1, 2, 3 (`PWM1` Ch 0..2)
//! - **D3 (`P0.29`)**: Motor 1 ESC (`PWM0` Ch 0)
//! - **D4, D5 (`P0.04`, `P0.05`)**: External I2C Bus (`TWISPI1` SDA/SCL for Compass & Barometer)
//! - **D6, D7 (`P1.11`, `P1.12`)**: RC Radio Link (`UARTE0` TX/RX for CRSF / iBUS)
//! - **D8 (`P1.13`)**: Motor 2 ESC (`PWM0` Ch 1, Twin Motor / Differential Thrust)
//! - **D9 (`P1.14`)**: Servo 4 (`PWM1` Ch 3, Secondary Aileron / Flap / Aux)
//! - **D10 (`P1.15`)**: Servo 5 (`PWM2` Ch 0, Aux / Pan / Gear)
//!
//! #### XIAO Sense Plus (23 Castellated Pads: D0..D19, 5V, GND, 3V3):
//! - **D0..D10**: Shared identical pinout with Standard Sense
//! - **D11..D13 (`P0.15`, `P0.19`, `P1.01`)**: Servos 6, 7, 8 (`PWM2` Ch 1..3)
//! - **D14, D15 (`P0.09`, `P0.10`)**: GPS Serial Link (`UARTE1` RX/TX)
//! - **D16 (`P0.31`)**: Battery Voltage ADC (`SAADC` AIN7)
//! - **D17..D19 (`P1.03`, `P1.05`, `P1.07`)**: Expansion SPI / Spare GPIOs

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
    /// Physical Pin: P1.13 (Silkscreen: D8) - Motor 2 ESC signal (Twin Motor / Diff Thrust)
    pub pin_motor2: Peri<'static, peripherals::P1_13>,

    // ==========================================
    // Actuators (PWM1: Primary Servos 1..4)
    // ==========================================
    /// PWM1 peripheral dedicated to primary 4 servo outputs (50Hz..400Hz)
    pub pwm1_servos: Peri<'static, peripherals::PWM1>,
    /// Physical Pin: P0.02 (Silkscreen: D0) - Servo 1 (Left Elevon / Primary Aileron)
    pub pin_servo1: Peri<'static, peripherals::P0_02>,
    /// Physical Pin: P0.03 (Silkscreen: D1) - Servo 2 (Right Elevon / Elevator / Right V-Tail)
    pub pin_servo2: Peri<'static, peripherals::P0_03>,
    /// Physical Pin: P0.28 (Silkscreen: D2) - Servo 3 (Rudder / Left V-Tail / Aux)
    pub pin_servo3: Peri<'static, peripherals::P0_28>,
    /// Physical Pin: P1.14 (Silkscreen: D9) - Servo 4 (Secondary Aileron / Flap / Aux)
    pub pin_servo4: Peri<'static, peripherals::P1_14>,

    // ==========================================
    // Actuators (PWM2: Secondary Servos 5..8)
    // ==========================================
    #[cfg(feature = "gps")]
    /// PWM2 peripheral dedicated to secondary servo outputs (Sense Plus variant)
    pub pwm2_servos: Peri<'static, peripherals::PWM2>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.15 (Silkscreen: D10) - Servo 5 (Aux / Flap / Gear)
    pub pin_servo5: Peri<'static, peripherals::P1_15>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P0.15 (Silkscreen: D11) - Servo 6 (Aux / Pan)
    pub pin_servo6: Peri<'static, peripherals::P0_15>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P0.19 (Silkscreen: D12) - Servo 7 (Aux / Tilt)
    pub pin_servo7: Peri<'static, peripherals::P0_19>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P1.01 (Silkscreen: D13) - Servo 8 (Aux / Drop)
    pub pin_servo8: Peri<'static, peripherals::P1_01>,

    // ==========================================
    // External Sensor Bus (TWISPI1: Compass & Barometer)
    // ==========================================
    /// TWISPI1 hardware peripheral for external I2C bus (Compass / Barometer)
    pub twispi_ext: Peri<'static, peripherals::TWISPI1>,
    /// Physical Pin: P0.04 (Silkscreen: D4) - External I2C SDA
    pub pin_ext_sda: Peri<'static, peripherals::P0_04>,
    /// Physical Pin: P0.05 (Silkscreen: D5) - External I2C SCL
    pub pin_ext_scl: Peri<'static, peripherals::P0_05>,

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
    /// Physical Pin: P0.09 (Silkscreen: D14 / RX1) - GPS UART RX
    pub pin_gps_rx: Peri<'static, peripherals::P0_09>,
    #[cfg(feature = "gps")]
    /// Physical Pin: P0.10 (Silkscreen: D15 / TX1) - GPS UART TX
    pub pin_gps_tx: Peri<'static, peripherals::P0_10>,

    // ==========================================
    // On-Board QSPI NOR Flash (Puya P25Q16H 2MB)
    // ==========================================
    /// Hardware QSPI peripheral for high-speed DMA flash logging
    pub qspi: Peri<'static, peripherals::QSPI>,
    /// Physical Pin: P0.21 (Internal) - QSPI SCK
    pub pin_qspi_sck: Peri<'static, peripherals::P0_21>,
    /// Physical Pin: P0.25 (Internal) - QSPI CSN
    pub pin_qspi_csn: Peri<'static, peripherals::P0_25>,
    /// Physical Pin: P0.20 (Internal) - QSPI IO0 (MOSI)
    pub pin_qspi_io0: Peri<'static, peripherals::P0_20>,
    /// Physical Pin: P0.24 (Internal) - QSPI IO1 (MISO)
    pub pin_qspi_io1: Peri<'static, peripherals::P0_24>,
    /// Physical Pin: P0.22 (Internal) - QSPI IO2 (WP)
    pub pin_qspi_io2: Peri<'static, peripherals::P0_22>,
    /// Physical Pin: P0.23 (Internal) - QSPI IO3 (HOLD)
    pub pin_qspi_io3: Peri<'static, peripherals::P0_23>,

    // ==========================================
    // USB Device (CDC-ACM Log Offload)
    // ==========================================
    /// Hardware USBD peripheral for disarmed blackbox log extraction
    pub usbd: Peri<'static, peripherals::USBD>,
}

impl BoardPins {
    /// Extracts and groups pins from the initialized `embassy_nrf::Peripherals`.
    pub fn from_peripherals(p: embassy_nrf::Peripherals) -> Self {
        Self {
            wdt: p.WDT,
            pwm_esc: p.PWM0,
            pin_motor1: p.P0_29,
            pin_motor2: p.P1_13,
            pwm1_servos: p.PWM1,
            pin_servo1: p.P0_02,
            pin_servo2: p.P0_03,
            pin_servo3: p.P0_28,
            pin_servo4: p.P1_14,
            #[cfg(feature = "gps")]
            pwm2_servos: p.PWM2,
            #[cfg(feature = "gps")]
            pin_servo5: p.P1_15,
            #[cfg(feature = "gps")]
            pin_servo6: p.P0_15,
            #[cfg(feature = "gps")]
            pin_servo7: p.P0_19,
            #[cfg(feature = "gps")]
            pin_servo8: p.P1_01,
            twispi_ext: p.TWISPI1,
            pin_ext_sda: p.P0_04,
            pin_ext_scl: p.P0_05,
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
            qspi: p.QSPI,
            pin_qspi_sck: p.P0_21,
            pin_qspi_csn: p.P0_25,
            pin_qspi_io0: p.P0_20,
            pin_qspi_io1: p.P0_24,
            pin_qspi_io2: p.P0_22,
            pin_qspi_io3: p.P0_23,
            usbd: p.USBD,
        }
    }
}
