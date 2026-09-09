//! External Magnetometer / Compass drivers (QMC5883L & HMC5883L) on TWISPI1.
//!
//! Provides support for explicit external I2C magnetometers on pins D4 (P0.04 SDA)
//! and D5 (P0.05 SCL). No auto-detection is performed to prevent dead code or runtime ambiguity.

use embassy_nrf::twim::Twim;
use embassy_time::{with_timeout, Duration, Timer};
use wingfc_core::math::Vector3;

/// QMC5883L Default 7-bit I2C Address
pub const QMC5883L_ADDR: u8 = 0x0D;

/// HMC5883L Default 7-bit I2C Address
pub const HMC5883L_ADDR: u8 = 0x1E;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CompassModel {
    Qmc5883l,
    Hmc5883l,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum CompassError {
    DeviceNotFound,
    I2cError,
    DataNotReady,
}

// =============================================================================
// QMC5883L Driver Implementation
// =============================================================================
// Registers:
// 0x00..0x05: Output Data (X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB)
// 0x06: Status Register (Bit 0: DRDY, Bit 1: OVL, Bit 2: DOR)
// 0x09: Control Register 1 (OSR 512, 8G Range, 50Hz ODR, Continuous Mode -> 0x15 or 100Hz -> 0x19)
// 0x0A: Control Register 2 (Soft Reset, Pointer Roll-over)
// 0x0B: SET/RESET Period Register (Datasheet recommended value: 0x01)
// 0x0D: Chip ID Register (Typically 0xFF)

const QMC_REG_DATA_OUT: u8 = 0x00;
const QMC_REG_STATUS: u8 = 0x06;
const QMC_REG_CTRL1: u8 = 0x09;
const QMC_REG_CTRL2: u8 = 0x0A;
const QMC_REG_SET_RESET: u8 = 0x0B;

pub struct Qmc5883lDriver<'d> {
    twim: Twim<'d>,
}

impl<'d> Qmc5883lDriver<'d> {
    pub fn new(twim: Twim<'d>) -> Self {
        Self { twim }
    }

    pub async fn init(&mut self) -> Result<(), CompassError> {
        Timer::after(Duration::from_millis(20)).await;

        // Reset QMC5883L via Control Register 2 (0x80 = SOFT_RST)
        let reset_cmd = [QMC_REG_CTRL2, 0x80];
        let _ = with_timeout(
            Duration::from_millis(50),
            self.twim.write(QMC5883L_ADDR, &reset_cmd),
        )
        .await;

        Timer::after(Duration::from_millis(20)).await;

        // Recommended Set/Reset period: 0x01
        let set_reset_cmd = [QMC_REG_SET_RESET, 0x01];
        with_timeout(
            Duration::from_millis(50),
            self.twim.write(QMC5883L_ADDR, &set_reset_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Configure Control Register 1:
        // OSR = 512 (0b00 << 6)
        // RNG = +/- 8 Gauss (0b01 << 4) -> 0x10
        // ODR = 100 Hz (0b10 << 2) -> 0x08
        // MODE = Continuous (0b01) -> 0x01
        // Value = 0b00011001 = 0x19
        let ctrl1_cmd = [QMC_REG_CTRL1, 0x19];
        with_timeout(
            Duration::from_millis(50),
            self.twim.write(QMC5883L_ADDR, &ctrl1_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Enable interrupt / rollover in Control Register 2: 0x40 (Pointer roll-over enabled)
        let ctrl2_cmd = [QMC_REG_CTRL2, 0x40];
        with_timeout(
            Duration::from_millis(50),
            self.twim.write(QMC5883L_ADDR, &ctrl2_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        defmt::info!("QMC5883L Magnetometer initialized on external I2C (TWISPI1, addr 0x0D)");
        Ok(())
    }

    /// Read 3-axis magnetic flux density vector in micro-Tesla (uT).
    pub async fn read(&mut self) -> Result<Vector3, CompassError> {
        // Read status register (0x06) to verify DRDY
        let mut status = [0u8; 1];
        with_timeout(
            Duration::from_millis(10),
            self.twim
                .write_read(QMC5883L_ADDR, &[QMC_REG_STATUS], &mut status),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Bit 0 is DRDY (Data Ready)
        if (status[0] & 0x01) == 0 {
            return Err(CompassError::DataNotReady);
        }

        // Read 6 bytes of data starting at 0x00: X_L, X_H, Y_L, Y_H, Z_L, Z_H
        let mut buf = [0u8; 6];
        with_timeout(
            Duration::from_millis(10),
            self.twim
                .write_read(QMC5883L_ADDR, &[QMC_REG_DATA_OUT], &mut buf),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        let raw_x = i16::from_le_bytes([buf[0], buf[1]]);
        let raw_y = i16::from_le_bytes([buf[2], buf[3]]);
        let raw_z = i16::from_le_bytes([buf[4], buf[5]]);

        // At +/- 8G full scale, sensitivity is 3000 LSB/Gauss = 30 LSB/uT (1 Gauss = 100 uT)
        // Scale to micro-Tesla (uT)
        const SCALE_TO_UT: f32 = 100.0 / 3000.0; // ~0.0333 uT / LSB
        Ok(Vector3::new(
            (raw_x as f32) * SCALE_TO_UT,
            (raw_y as f32) * SCALE_TO_UT,
            (raw_z as f32) * SCALE_TO_UT,
        ))
    }
}

// =============================================================================
// HMC5883L Driver Implementation
// =============================================================================
// Registers:
// 0x00: Configuration Register A (CRA): Samples avg (8 = 0b11), ODR (75Hz = 0b110), Measurement (Normal = 0b00) -> 0x78
// 0x01: Configuration Register B (CRB): Gain +/- 1.3 Gauss (0b001 << 5) -> 0x20
// 0x02: Mode Register: Continuous Mode (0x00)
// 0x03..0x08: Output Data (X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB) -> Note order is X, Z, Y!
// 0x09: Status Register (Bit 0: RDY, Bit 1: LOCK)
// 0x0A..0x0C: Identification Registers A, B, C ('H', '4', '3' -> 0x48, 0x34, 0x33)

const HMC_REG_CRA: u8 = 0x00;
const HMC_REG_CRB: u8 = 0x01;
const HMC_REG_MODE: u8 = 0x02;
const HMC_REG_DATA_OUT: u8 = 0x03;
const HMC_REG_STATUS: u8 = 0x09;
const HMC_REG_IDA: u8 = 0x0A;

pub struct Hmc5883lDriver<'d> {
    twim: Twim<'d>,
}

impl<'d> Hmc5883lDriver<'d> {
    pub fn new(twim: Twim<'d>) -> Self {
        Self { twim }
    }

    pub async fn init(&mut self) -> Result<(), CompassError> {
        Timer::after(Duration::from_millis(20)).await;

        // Verify Identification Register A ('H' = 0x48)
        let mut id = [0u8; 3];
        let id_res = with_timeout(
            Duration::from_millis(50),
            self.twim.write_read(HMC5883L_ADDR, &[HMC_REG_IDA], &mut id),
        )
        .await;

        match id_res {
            Ok(Ok(())) => {
                if id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33 {
                    defmt::warn!("HMC5883L ID mismatch: expected H43, got {=[u8]:x}", id);
                }
            }
            _ => return Err(CompassError::DeviceNotFound),
        }

        // Configure CRA: 8 samples averaged, 75 Hz data rate, normal measurement mode (0x78)
        let cra_cmd = [HMC_REG_CRA, 0x78];
        with_timeout(
            Duration::from_millis(50),
            self.twim.write(HMC5883L_ADDR, &cra_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Configure CRB: Gain = +/- 1.3 Gauss (1090 LSB / Gauss -> 0x20)
        let crb_cmd = [HMC_REG_CRB, 0x20];
        with_timeout(
            Duration::from_millis(50),
            self.twim.write(HMC5883L_ADDR, &crb_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Configure Mode: Continuous measurement mode (0x00)
        let mode_cmd = [HMC_REG_MODE, 0x00];
        with_timeout(
            Duration::from_millis(50),
            self.twim.write(HMC5883L_ADDR, &mode_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        defmt::info!("HMC5883L Magnetometer initialized on external I2C (TWISPI1, addr 0x1E)");
        Ok(())
    }

    /// Read 3-axis magnetic flux density vector in micro-Tesla (uT).
    pub async fn read(&mut self) -> Result<Vector3, CompassError> {
        // Read status register (0x09) to verify RDY bit
        let mut status = [0u8; 1];
        with_timeout(
            Duration::from_millis(10),
            self.twim
                .write_read(HMC5883L_ADDR, &[HMC_REG_STATUS], &mut status),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Bit 0 is RDY (Data Ready)
        if (status[0] & 0x01) == 0 {
            return Err(CompassError::DataNotReady);
        }

        // Read 6 bytes of data starting at 0x03: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
        let mut buf = [0u8; 6];
        with_timeout(
            Duration::from_millis(10),
            self.twim
                .write_read(HMC5883L_ADDR, &[HMC_REG_DATA_OUT], &mut buf),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        let raw_x = i16::from_be_bytes([buf[0], buf[1]]);
        let raw_z = i16::from_be_bytes([buf[2], buf[3]]);
        let raw_y = i16::from_be_bytes([buf[4], buf[5]]);

        // At +/- 1.3G full scale, sensitivity is 1090 LSB/Gauss (1 Gauss = 100 uT)
        // Scale to micro-Tesla (uT)
        const SCALE_TO_UT: f32 = 100.0 / 1090.0; // ~0.0917 uT / LSB
        Ok(Vector3::new(
            (raw_x as f32) * SCALE_TO_UT,
            (raw_y as f32) * SCALE_TO_UT,
            (raw_z as f32) * SCALE_TO_UT,
        ))
    }
}
