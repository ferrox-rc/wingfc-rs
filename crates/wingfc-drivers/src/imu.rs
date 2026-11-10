//! Hardware-agnostic driver for the STMicroelectronics LSM6DS3TR-C 6-DoF IMU.
//!
//! Built on `embedded_hal_async::i2c::I2c` and `embedded_hal_async::digital::Wait` so the exact
//! same register initialization, DRDY interrupt pacing, 8:1 boxcar decimation, and thermal readouts
//! work unchanged across Nordic nRF52840 (`TWIM` + `GPIOTE`) and STM32C5 (`I2C2` + `EXTI`).

use embassy_time::{with_timeout, Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use wingfc_core::downsample::{BoxcarAccumulator8, ImuDecimated};
use wingfc_core::math::{BoardOrientation, Vector3};

pub const LSM6DS3_ADDR: u8 = 0x6A;

// Register definitions
const INT1_CTRL_REG: u8 = 0x0D;
const WHO_AM_I_REG: u8 = 0x0F;
const CTRL1_XL_REG: u8 = 0x10;
const CTRL2_G_REG: u8 = 0x11;
const CTRL3_C_REG: u8 = 0x12;
const OUT_TEMP_L_REG: u8 = 0x20;
const OUTX_L_G_REG: u8 = 0x22;

// Sensitivity constants
// Gyro at +/- 2000 dps: 70 mdps/LSB -> rad/s
const GYRO_SCALE_2000DPS_TO_RADS: f32 = 70.0 * 1e-3 * (core::f32::consts::PI / 180.0);
// Accel at +/- 16g: 0.488 mg/LSB -> m/s^2
const ACCEL_SCALE_16G_TO_MS2: f32 = 0.488 * 1e-3 * 9.80665;
// Temperature sensitivity: 256 LSB/degC, zero offset = 25.0 degC
const TEMP_SCALE_LSB_TO_C: f32 = 1.0 / 256.0;
const TEMP_OFFSET_C: f32 = 25.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ImuError {
    DeviceNotFound,
    I2cError,
}

/// Static configuration parameters for [`Lsm6ds3Driver`].
#[derive(Debug, Clone, Copy)]
pub struct Lsm6ds3Config {
    /// Board mounting orientation transform
    pub orientation: BoardOrientation,
    /// `CTRL1_XL` register byte (e.g. `0x84` for 1.66 kHz ODR, +/-16g)
    pub odr_xl_reg_val: u8,
    /// `CTRL2_G` register byte (e.g. `0x8C` for 1.66 kHz ODR, +/-2000 dps)
    pub odr_g_reg_val: u8,
    /// Number of raw samples boxcar-averaged per frame (1 = raw, 8 = 8:1 decimation)
    pub decimation: u8,
    /// Nominal flight loop timestep in seconds
    pub nominal_dt: f32,
    /// Nominal flight loop frequency in Hz (for diagnostic logging)
    pub nominal_loop_hz: u32,
}

impl Default for Lsm6ds3Config {
    fn default() -> Self {
        Self {
            orientation: BoardOrientation::Default,
            odr_xl_reg_val: 0x84,
            odr_g_reg_val: 0x8C,
            decimation: 8,
            nominal_dt: 0.0048,
            nominal_loop_hz: 208,
        }
    }
}

pub struct Lsm6ds3Driver<I2C, INT> {
    i2c: I2C,
    int1: INT,
    config: Lsm6ds3Config,
    gyro_bias: Vector3,
    /// 8:1 boxcar accumulator (used when `config.decimation > 1`)
    accumulator: BoxcarAccumulator8,
    /// Latest decimated frame, or `None` if a sample read failed
    frame: Option<ImuDecimated>,
    raw_seq: u32,
}

impl<I2C, INT> Lsm6ds3Driver<I2C, INT>
where
    I2C: I2c,
    INT: Wait,
{
    pub fn new(i2c: I2C, int1: INT, config: Lsm6ds3Config) -> Self {
        Self {
            i2c,
            int1,
            config,
            gyro_bias: Vector3::ZERO,
            accumulator: BoxcarAccumulator8::new(),
            frame: None,
            raw_seq: 1,
        }
    }

    pub async fn init(&mut self) -> Result<(), ImuError> {
        Timer::after(Duration::from_millis(20)).await;

        // Verify WHO_AM_I with timeout
        let mut who_am_i = [0u8; 1];
        with_timeout(
            Duration::from_millis(50),
            self.i2c
                .write_read(LSM6DS3_ADDR, &[WHO_AM_I_REG], &mut who_am_i),
        )
        .await
        .map_err(|_| ImuError::I2cError)?
        .map_err(|_| ImuError::I2cError)?;

        if who_am_i[0] != 0x69 && who_am_i[0] != 0x6A {
            #[cfg(feature = "defmt-03")]
            defmt::error!("Unexpected WHO_AM_I: 0x{:02X}", who_am_i[0]);
            return Err(ImuError::DeviceNotFound);
        }

        // CTRL3_C: Enable Block Data Update (BDU) and register address auto-increment
        self.i2c
            .write(LSM6DS3_ADDR, &[CTRL3_C_REG, 0x44])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // CTRL1_XL: Accel ODR = configured ODR, Full-Scale = +/- 16g
        self.i2c
            .write(LSM6DS3_ADDR, &[CTRL1_XL_REG, self.config.odr_xl_reg_val])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // CTRL2_G: Gyro ODR = configured ODR, Full-Scale = +/- 2000 dps
        self.i2c
            .write(LSM6DS3_ADDR, &[CTRL2_G_REG, self.config.odr_g_reg_val])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // INT1_CTRL: Route Gyro Data Ready (INT1_DRDY_G = 0x02) to INT1 hardware pin
        self.i2c
            .write(LSM6DS3_ADDR, &[INT1_CTRL_REG, 0x02])
            .await
            .map_err(|_| ImuError::I2cError)?;

        #[cfg(feature = "defmt-03")]
        defmt::info!(
            "LSM6DS3TR IMU initialized (loop {}Hz, decimation {}:1, DRDY on INT1, +/-2000 dps, +/-16g)",
            self.config.nominal_loop_hz,
            self.config.decimation
        );
        Ok(())
    }

    /// Asynchronously awaits the LSM6DS3 hardware data-ready signal on INT1.
    ///
    /// Suspends the CPU in low-power WFE sleep until the INT1 pin asserts HIGH, indicating
    /// new Gyro and Accel sample data is latched in hardware registers.
    ///
    /// When `config.decimation > 1`, this waits for and reads `decimation` consecutive
    /// samples, averaging them into a single frame returned by [`Self::read`].
    pub async fn wait_data_ready(&mut self) {
        if self.config.decimation <= 1 {
            let _ = self.int1.wait_for_high().await;
            return;
        }
        loop {
            let _ = self.int1.wait_for_high().await;
            match self.read_sample().await {
                Ok((gyro, accel)) => {
                    if let Some(f) = self.accumulator.push(accel, gyro) {
                        self.frame = Some(f);
                        return;
                    }
                }
                Err(_) => {
                    self.accumulator.reset();
                    self.frame = None;
                    return;
                }
            }
        }
    }

    /// Calibrates gyroscope offsets by averaging stationary samples.
    pub async fn calibrate_gyro(&mut self, sample_count: usize) -> Result<(), ImuError> {
        #[cfg(feature = "defmt-03")]
        defmt::info!("Calibrating gyro (keep aircraft stationary)...");
        let mut sum_x = 0.0f32;
        let mut sum_y = 0.0f32;
        let mut sum_z = 0.0f32;

        for _ in 0..sample_count {
            let (gyro, _) = self.read_sample().await?;
            sum_x += gyro.x;
            sum_y += gyro.y;
            sum_z += gyro.z;
        }

        let inv = 1.0 / (sample_count as f32);
        self.gyro_bias = Vector3::new(sum_x * inv, sum_y * inv, sum_z * inv);
        #[cfg(feature = "defmt-03")]
        defmt::info!(
            "Gyro calibration complete: x={}, y={}, z={}",
            self.gyro_bias.x,
            self.gyro_bias.y,
            self.gyro_bias.z
        );
        Ok(())
    }

    /// Returns the latest `ImuDecimated` sample frame containing `(accel, gyro, seq)`.
    ///
    /// Without decimation this burst-reads 12 bytes over DMA I2C. With decimation it returns
    /// the boxcar-averaged frame produced by [`Self::wait_data_ready`] without touching the bus.
    pub async fn read(&mut self) -> Result<ImuDecimated, ImuError> {
        if self.config.decimation > 1 {
            return self.frame.take().ok_or(ImuError::I2cError);
        }
        let (gyro, accel) = self.read_sample().await?;
        let seq = self.raw_seq;
        self.raw_seq = self.raw_seq.wrapping_add(1);
        let dt = self.config.nominal_dt;
        Ok(ImuDecimated {
            accel,
            gyro,
            dt,
            seq,
        })
    }

    /// Awaits the LSM6DS3 hardware data-ready signal (or 8:1 decimation window) and reads the latest frame.
    pub async fn next_frame(&mut self) -> Result<ImuDecimated, ImuError> {
        self.wait_data_ready().await;
        self.read().await
    }

    /// Reads the internal IMU die temperature in degrees Celsius (`OUT_TEMP_L/H` at `0x20..0x21`).
    ///
    /// Used by boards with onboard IMU thermal control (such as XIAO STM32C5's `PA8` heater PWM)
    /// to regulate die temperature and eliminate thermal gyro bias drift.
    pub async fn read_temperature_c(&mut self) -> Result<f32, ImuError> {
        let mut buf = [0u8; 2];
        let reg = [OUT_TEMP_L_REG];
        with_timeout(
            Duration::from_millis(5),
            self.i2c.write_read(LSM6DS3_ADDR, &reg, &mut buf),
        )
        .await
        .map_err(|_| ImuError::I2cError)?
        .map_err(|_| ImuError::I2cError)?;

        let raw_temp = i16::from_le_bytes(buf);
        Ok(TEMP_OFFSET_C + (raw_temp as f32) * TEMP_SCALE_LSB_TO_C)
    }

    /// Reads one raw sample (bias-corrected, body-frame) as `(gyro, accel)`.
    async fn read_sample(&mut self) -> Result<(Vector3, Vector3), ImuError> {
        let (raw_gyro, accel) = self.read_raw_scaled().await?;
        // Gyro calibration offsets are captured directly from sensor registers in the physical sensor frame.
        // Therefore, bias MUST be subtracted in the raw sensor frame BEFORE applying BOARD_ORIENTATION
        // body-frame coordinate transformations.
        let gyro = raw_gyro - self.gyro_bias;
        let gyro = self.config.orientation.transform(gyro);
        let accel = self.config.orientation.transform(accel);
        Ok((gyro, accel))
    }

    async fn read_raw_scaled(&mut self) -> Result<(Vector3, Vector3), ImuError> {
        // Read 12 contiguous bytes: Gyro X, Y, Z (6 bytes) followed by Accel X, Y, Z (6 bytes)
        // using non-blocking hardware DMA.
        // Use stack-allocated buffer for TX to ensure RAM address for DMA controllers.
        let mut buf = [0u8; 12];
        let reg = [OUTX_L_G_REG];
        with_timeout(
            Duration::from_millis(5),
            self.i2c.write_read(LSM6DS3_ADDR, &reg, &mut buf),
        )
        .await
        .map_err(|_| ImuError::I2cError)?
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
