use embassy_nrf::twim::{self, Twim};
use embassy_time::{Duration, Timer};
use wingfc_core::math::Vector3;

pub const LSM6DS3_ADDR: u8 = 0x6A;

// Register definitions
const WHO_AM_I_REG: u8 = 0x0F;
const CTRL1_XL_REG: u8 = 0x10;
const CTRL2_G_REG: u8 = 0x11;
const CTRL3_C_REG: u8 = 0x12;
const OUTX_L_G_REG: u8 = 0x22;

// Sensitivity constants
// Gyro at +/- 2000 dps: 70 mdps/LSB -> rad/s
const GYRO_SCALE_2000DPS_TO_RADS: f32 = 70.0 * 1e-3 * (core::f32::consts::PI / 180.0);
// Accel at +/- 16g: 0.488 mg/LSB -> m/s^2
const ACCEL_SCALE_16G_TO_MS2: f32 = 0.488 * 1e-3 * 9.80665;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ImuError {
    DeviceNotFound,
    I2cError,
}

pub struct Lsm6ds3Driver<'d, T: twim::Instance> {
    twim: Twim<'d, T>,
    gyro_bias: Vector3,
}

impl<'d, T: twim::Instance> Lsm6ds3Driver<'d, T> {
    pub fn new(twim: Twim<'d, T>) -> Self {
        Self {
            twim,
            gyro_bias: Vector3::ZERO,
        }
    }

    pub async fn init(&mut self) -> Result<(), ImuError> {
        Timer::after(Duration::from_millis(15)).await;

        // Verify WHO_AM_I
        let mut who_am_i = [0u8; 1];
        self.twim
            .write_read(LSM6DS3_ADDR, &[WHO_AM_I_REG], &mut who_am_i)
            .await
            .map_err(|_| ImuError::I2cError)?;

        if who_am_i[0] != 0x69 && who_am_i[0] != 0x6A {
            defmt::error!("Unexpected WHO_AM_I: 0x{:02X}", who_am_i[0]);
            return Err(ImuError::DeviceNotFound);
        }

        // CTRL3_C: Enable Block Data Update (BDU) and register address auto-increment
        self.twim
            .write(LSM6DS3_ADDR, &[CTRL3_C_REG, 0x44])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // CTRL1_XL: Accel ODR = 1.66 kHz, Full-Scale = +/- 16g (0x80 | 0x04 = 0x84)
        self.twim
            .write(LSM6DS3_ADDR, &[CTRL1_XL_REG, 0x84])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // CTRL2_G: Gyro ODR = 1.66 kHz, Full-Scale = +/- 2000 dps (0x80 | 0x0C = 0x8C)
        self.twim
            .write(LSM6DS3_ADDR, &[CTRL2_G_REG, 0x8C])
            .await
            .map_err(|_| ImuError::I2cError)?;

        defmt::info!("LSM6DS3TR IMU initialized @ 1.66kHz ODR (+/-2000 dps, +/-16g)");
        Ok(())
    }

    /// Calibrates gyroscope offsets by averaging stationary samples
    pub async fn calibrate_gyro(&mut self, sample_count: usize) -> Result<(), ImuError> {
        defmt::info!("Calibrating gyro (keep aircraft stationary)...");
        let mut sum_x = 0.0f32;
        let mut sum_y = 0.0f32;
        let mut sum_z = 0.0f32;

        for _ in 0..sample_count {
            let (gyro, _) = self.read_raw_scaled().await?;
            sum_x += gyro.x;
            sum_y += gyro.y;
            sum_z += gyro.z;
            Timer::after(Duration::from_millis(2)).await;
        }

        let inv = 1.0 / (sample_count as f32);
        self.gyro_bias = Vector3::new(sum_x * inv, sum_y * inv, sum_z * inv);
        defmt::info!(
            "Gyro calibration complete: x={}, y={}, z={}",
            self.gyro_bias.x,
            self.gyro_bias.y,
            self.gyro_bias.z
        );
        Ok(())
    }

    /// Reads Gyro (rad/s) and Accel (m/s^2) in a single burst read
    pub async fn read(&mut self) -> Result<(Vector3, Vector3), ImuError> {
        let (raw_gyro, accel) = self.read_raw_scaled().await?;
        let gyro = raw_gyro - self.gyro_bias;
        Ok((gyro, accel))
    }

    async fn read_raw_scaled(&mut self) -> Result<(Vector3, Vector3), ImuError> {
        // Read 12 contiguous bytes: Gyro X, Y, Z (6 bytes) followed by Accel X, Y, Z (6 bytes)
        let mut buf = [0u8; 12];
        self.twim
            .write_read(LSM6DS3_ADDR, &[OUTX_L_G_REG], &mut buf)
            .await
            .map_err(|_| ImuError::I2cError)?;

        let gx = (buf[0] as i16 | ((buf[1] as i16) << 8)) as f32 * GYRO_SCALE_2000DPS_TO_RADS;
        let gy = (buf[2] as i16 | ((buf[3] as i16) << 8)) as f32 * GYRO_SCALE_2000DPS_TO_RADS;
        let gz = (buf[4] as i16 | ((buf[5] as i16) << 8)) as f32 * GYRO_SCALE_2000DPS_TO_RADS;

        let ax = (buf[6] as i16 | ((buf[7] as i16) << 8)) as f32 * ACCEL_SCALE_16G_TO_MS2;
        let ay = (buf[8] as i16 | ((buf[9] as i16) << 8)) as f32 * ACCEL_SCALE_16G_TO_MS2;
        let az = (buf[10] as i16 | ((buf[11] as i16) << 8)) as f32 * ACCEL_SCALE_16G_TO_MS2;

        Ok((Vector3::new(gx, gy, gz), Vector3::new(ax, ay, az)))
    }
}
