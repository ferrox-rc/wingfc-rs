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
    I2cError,
    DataNotReady,
}

// =============================================================================
// QMC5883L Driver Implementation
// =============================================================================
const QMC_REG_DATA_OUT: u8 = 0x00;
const QMC_REG_STATUS: u8 = 0x06;
const QMC_REG_CTRL1: u8 = 0x09;
const QMC_REG_CTRL2: u8 = 0x0A;
const QMC_REG_SET_RESET: u8 = 0x0B;

pub struct Qmc5883lDriver {
    addr: u8,
}

impl Default for Qmc5883lDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl Qmc5883lDriver {
    pub fn new() -> Self {
        Self {
            addr: QMC5883L_ADDR,
        }
    }

    pub async fn init(&mut self, twim: &mut Twim<'_>) -> Result<(), CompassError> {
        Timer::after(Duration::from_millis(20)).await;

        // Reset QMC5883L via Control Register 2 (0x80 = SOFT_RST)
        let reset_cmd = [QMC_REG_CTRL2, 0x80];
        let _ = with_timeout(Duration::from_millis(50), twim.write(self.addr, &reset_cmd)).await;

        Timer::after(Duration::from_millis(20)).await;

        // Recommended Set/Reset period: 0x01
        let set_reset_cmd = [QMC_REG_SET_RESET, 0x01];
        with_timeout(
            Duration::from_millis(50),
            twim.write(self.addr, &set_reset_cmd),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        // Control Register 1: 50Hz, 8G, continuous
        let ctrl1_cmd = [QMC_REG_CTRL1, 0x15];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &ctrl1_cmd))
            .await
            .map_err(|_| CompassError::I2cError)?
            .map_err(|_| CompassError::I2cError)?;

        defmt::info!(
            "QMC5883L Magnetometer initialized on external I2C (addr 0x{:02X})",
            self.addr
        );
        Ok(())
    }

    /// Read 3-axis magnetic flux density vector in micro-Tesla (uT).
    pub async fn read(&mut self, twim: &mut Twim<'_>) -> Result<Vector3, CompassError> {
        let mut status = [0u8; 1];
        with_timeout(
            Duration::from_millis(10),
            twim.write_read(self.addr, &[QMC_REG_STATUS], &mut status),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        if (status[0] & 0x01) == 0 {
            return Err(CompassError::DataNotReady);
        }

        let mut buf = [0u8; 6];
        with_timeout(
            Duration::from_millis(10),
            twim.write_read(self.addr, &[QMC_REG_DATA_OUT], &mut buf),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        let raw_x = i16::from_le_bytes([buf[0], buf[1]]);
        let raw_y = i16::from_le_bytes([buf[2], buf[3]]);
        let raw_z = i16::from_le_bytes([buf[4], buf[5]]);

        const SCALE_TO_UT: f32 = 100.0 / 3000.0;
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
const HMC_REG_CRA: u8 = 0x00;
const HMC_REG_CRB: u8 = 0x01;
const HMC_REG_MODE: u8 = 0x02;
const HMC_REG_DATA_OUT: u8 = 0x03;
const HMC_REG_STATUS: u8 = 0x09;

pub struct Hmc5883lDriver {
    addr: u8,
}

impl Default for Hmc5883lDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl Hmc5883lDriver {
    pub fn new() -> Self {
        Self {
            addr: HMC5883L_ADDR,
        }
    }

    pub async fn init(&mut self, twim: &mut Twim<'_>) -> Result<(), CompassError> {
        Timer::after(Duration::from_millis(20)).await;

        let cra_cmd = [HMC_REG_CRA, 0x78];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &cra_cmd))
            .await
            .map_err(|_| CompassError::I2cError)?
            .map_err(|_| CompassError::I2cError)?;

        let crb_cmd = [HMC_REG_CRB, 0x20];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &crb_cmd))
            .await
            .map_err(|_| CompassError::I2cError)?
            .map_err(|_| CompassError::I2cError)?;

        let mode_cmd = [HMC_REG_MODE, 0x00];
        with_timeout(Duration::from_millis(50), twim.write(self.addr, &mode_cmd))
            .await
            .map_err(|_| CompassError::I2cError)?
            .map_err(|_| CompassError::I2cError)?;

        defmt::info!(
            "HMC5883L Magnetometer initialized on external I2C (addr 0x{:02X})",
            self.addr
        );
        Ok(())
    }

    /// Read 3-axis magnetic flux density vector in micro-Tesla (uT).
    pub async fn read(&mut self, twim: &mut Twim<'_>) -> Result<Vector3, CompassError> {
        let mut status = [0u8; 1];
        with_timeout(
            Duration::from_millis(10),
            twim.write_read(self.addr, &[HMC_REG_STATUS], &mut status),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        if (status[0] & 0x01) == 0 {
            return Err(CompassError::DataNotReady);
        }

        let mut buf = [0u8; 6];
        with_timeout(
            Duration::from_millis(10),
            twim.write_read(self.addr, &[HMC_REG_DATA_OUT], &mut buf),
        )
        .await
        .map_err(|_| CompassError::I2cError)?
        .map_err(|_| CompassError::I2cError)?;

        let raw_x = i16::from_be_bytes([buf[0], buf[1]]);
        let raw_z = i16::from_be_bytes([buf[2], buf[3]]);
        let raw_y = i16::from_be_bytes([buf[4], buf[5]]);

        const SCALE_TO_UT: f32 = 100.0 / 1090.0;
        Ok(Vector3::new(
            (raw_x as f32) * SCALE_TO_UT,
            (raw_y as f32) * SCALE_TO_UT,
            (raw_z as f32) * SCALE_TO_UT,
        ))
    }
}
