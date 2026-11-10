//! Seeed Studio XIAO STM32C5 Flight Controller Board Definition & Architecture.
//!
//! ### Hardware Overview:
//! - **MCU**: STMicroelectronics STM32C5A3CG (ARM Cortex-M33F @ 144 MHz, TrustZone, FPU)
//! - **Flash**: 1024 KB Flash, 256 KB SRAM
//! - **IMU**: ST LSM6DS3TR-C 6-DoF on internal `I2C2` (`PB4` SDA, `PB3` SCL) with `PC13` INT1
//! - **IMU Heater**: On-board thermal regulation via `PA8` (`TIM1_CH1` PWM), maintaining 45°C die temp
//! - **External NOR Flash**: 16 MB (128 Mbit) Puya PY25Q128HA on OctoSPI / XSPI (`PB1`, `PB2`, `PB10`, `PB11`, etc.)
//! - **External I2C**: `I2C1` on D4 (`PA0`) SDA, D5 (`PA1`) SCL for Barometer & Compass
//! - **Primary RC Serial**: `USART1` / `USART2` on D6 (`PA2`) TX, D7 (`PA3`) RX (CRSF 420k / iBUS 115.2k)
//! - **Secondary Serial (GPS)**: Secondary UART on D9/D10 or auxiliary pads
//! - **Actuators (Servos & Motors)**:
//!   - Motor 1 ESC: D3 (`PA15`) via TIM2_CH1
//!   - Motor 2 ESC: D8 (`PA7`) via TIM3_CH2 (Twin motor / Diff thrust)
//!   - Servos 1..4: D0 (`PA4`), D1 (`PA5`), D2 (`PA6`), D9 (`PA10`) via general-purpose timers
//! - **Telemetry Visuals**: Single yellow status LED on D13 / `PA9` (active LOW)
//! - **FDCAN**: Dual FDCAN controllers with onboard TCAN3403-Q1 CAN transceiver on `PB8`/`PB9`

use wingfc_core::math::BoardOrientation;

/// Total capacity of onboard PY25Q128HA NOR flash: 16 MB (16,777,216 bytes).
pub const STM32C5_FLASH_CAPACITY_BYTES: u32 = 16 * 1024 * 1024;

/// Target IMU die temperature setpoint in degrees Celsius for thermal drift elimination.
pub const IMU_TARGET_TEMP_C: f32 = 45.0;

/// Board identification string
pub const BOARD_NAME: &'static str = "Seeed Studio XIAO STM32C5";

/// Default IMU mounting orientation for the XIAO STM32C5 board
pub const DEFAULT_ORIENTATION: BoardOrientation = BoardOrientation::Default;

/// Target microcontroller architecture name
pub const TARGET_TRIPLE: &'static str = "thumbv8m.main-none-eabihf";

/// Structure representing physical hardware resources of the XIAO STM32C5.
///
/// Encapsulates the STM32C5 peripheral mapping when compiled with `board-xiao-stm32c5`.
pub struct XiaoStm32c5Board {
    pub imu_i2c_bus: u8,
    pub ext_i2c_bus: u8,
    pub rc_uart_bus: u8,
    pub heater_enabled: bool,
}

impl XiaoStm32c5Board {
    pub const fn new() -> Self {
        Self {
            imu_i2c_bus: 2,       // I2C2 on PB4/PB3
            ext_i2c_bus: 1,       // I2C1 on PA0/PA1
            rc_uart_bus: 1,       // USART1 on PA2/PA3
            heater_enabled: true, // PA8 TIM1_CH1 heater active
        }
    }
}

/// Simple PID controller for the on-board IMU heater (PA8 TIM1_CH1).
///
/// Holds the LSM6DS3TR-C at a constant 45°C to eliminate temperature-induced gyro bias drift.
pub struct ImuHeaterController {
    target_c: f32,
    kp: f32,
    ki: f32,
    integral: f32,
}

impl ImuHeaterController {
    pub const fn new(target_c: f32) -> Self {
        Self {
            target_c,
            kp: 0.15,
            ki: 0.02,
            integral: 0.0,
        }
    }

    /// Computes the heater PWM duty cycle (0.0 to 1.0) given current die temperature and dt.
    pub fn update(&mut self, current_temp_c: f32, dt: f32) -> f32 {
        let err = self.target_c - current_temp_c;
        if err > 0.0 {
            self.integral = (self.integral + err * dt).clamp(0.0, 1.0);
        } else {
            self.integral = (self.integral + err * dt).clamp(0.0, 1.0);
        }
        let output = self.kp * err + self.ki * self.integral;
        output.clamp(0.0, 1.0)
    }
}
