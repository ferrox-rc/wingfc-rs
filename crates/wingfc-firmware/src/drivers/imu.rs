use crate::config::{
    BOARD_ORIENTATION, IMU_DECIMATION, IMU_ODR_G_REG_VAL, IMU_ODR_XL_REG_VAL, NOMINAL_LOOP_HZ,
};
use embassy_nrf::gpio::Input;
use embassy_nrf::twim::Twim;
use embassy_time::{with_timeout, Duration, Timer};
use wingfc_core::downsample::{BoxcarAccumulator8, ImuDecimated};
use wingfc_core::math::Vector3;

pub const LSM6DS3_ADDR: u8 = 0x6A;

// Register definitions
const INT1_CTRL_REG: u8 = 0x0D;
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

pub struct Lsm6ds3Driver<'d> {
    twim: Twim<'d>,
    int1: Input<'d>,
    gyro_bias: Vector3,
    /// 8:1 boxcar accumulator (used only when `IMU_DECIMATION > 1`)
    accumulator: BoxcarAccumulator8,
    /// Latest decimated frame, or `None` if a sample read failed
    frame: Option<ImuDecimated>,
    raw_seq: u32,
}

impl<'d> Lsm6ds3Driver<'d> {
    pub fn new(twim: Twim<'d>, int1: Input<'d>) -> Self {
        Self {
            twim,
            int1,
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
            self.twim
                .write_read(LSM6DS3_ADDR, &[WHO_AM_I_REG], &mut who_am_i),
        )
        .await
        .map_err(|_| ImuError::I2cError)?
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

        // CTRL1_XL: Accel ODR = configured ODR, Full-Scale = +/- 16g
        self.twim
            .write(LSM6DS3_ADDR, &[CTRL1_XL_REG, IMU_ODR_XL_REG_VAL])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // CTRL2_G: Gyro ODR = configured ODR, Full-Scale = +/- 2000 dps
        self.twim
            .write(LSM6DS3_ADDR, &[CTRL2_G_REG, IMU_ODR_G_REG_VAL])
            .await
            .map_err(|_| ImuError::I2cError)?;

        // INT1_CTRL: Route Gyro Data Ready (INT1_DRDY_G = 0x02) to INT1 hardware pin (P0.11)
        self.twim
            .write(LSM6DS3_ADDR, &[INT1_CTRL_REG, 0x02])
            .await
            .map_err(|_| ImuError::I2cError)?;

        defmt::info!(
            "LSM6DS3TR IMU initialized (loop {}Hz, decimation {}:1, DRDY on INT1, +/-2000 dps, +/-16g)",
            NOMINAL_LOOP_HZ,
            IMU_DECIMATION
        );
        Ok(())
    }

    /// Asynchronously awaits the LSM6DS3 hardware data-ready signal on INT1 (P0.11) via GPIOTE.
    ///
    /// Suspends the CPU in low-power WFE sleep until the INT1 pin asserts HIGH, indicating
    /// new Gyro and Accel sample data is latched in hardware registers.
    ///
    /// When `IMU_DECIMATION > 1`, this waits for and reads `IMU_DECIMATION` consecutive
    /// samples, averaging them into a single frame returned by [`Self::read`]. The
    /// accumulator persists across cancellation (e.g. watchdog timeouts).
    pub async fn wait_data_ready(&mut self) {
        if IMU_DECIMATION <= 1 {
            self.int1.wait_for_high().await;
            return;
        }
        loop {
            self.int1.wait_for_high().await;
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

    /// Calibrates gyroscope offsets by averaging stationary samples
    pub async fn calibrate_gyro(&mut self, sample_count: usize) -> Result<(), ImuError> {
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
    /// Without decimation this burst-reads 12 bytes over 400kHz Fast-Mode EasyDMA I2C. With
    /// decimation it returns the boxcar-averaged frame produced by [`Self::wait_data_ready`]
    /// instead of touching the bus.
    pub async fn read(&mut self) -> Result<ImuDecimated, ImuError> {
        if IMU_DECIMATION > 1 {
            return self.frame.take().ok_or(ImuError::I2cError);
        }
        let (gyro, accel) = self.read_sample().await?;
        let seq = self.raw_seq;
        self.raw_seq = self.raw_seq.wrapping_add(1);
        let dt = crate::config::NOMINAL_DT;
        Ok(ImuDecimated { accel, gyro, dt, seq })
    }

    /// Awaits the LSM6DS3 hardware data-ready signal (or 8:1 decimation window) and reads the latest frame.
    pub async fn next_frame(&mut self) -> Result<ImuDecimated, ImuError> {
        self.wait_data_ready().await;
        self.read().await
    }

    /// Reads one raw sample (bias-corrected, body-frame) as `(gyro, accel)`.
    async fn read_sample(&mut self) -> Result<(Vector3, Vector3), ImuError> {
        let (raw_gyro, accel) = self.read_raw_scaled().await?;
        // Gyro calibration offsets are captured directly from sensor registers in the physical sensor frame.
        // Therefore, bias MUST be subtracted in the raw sensor frame BEFORE applying BOARD_ORIENTATION
        // body-frame coordinate transformations.
        let gyro = raw_gyro - self.gyro_bias;
        // Transform sensor axes to aircraft body frame based on board orientation
        let gyro = BOARD_ORIENTATION.transform(gyro);
        let accel = BOARD_ORIENTATION.transform(accel);
        Ok((gyro, accel))
    }

    async fn read_raw_scaled(&mut self) -> Result<(Vector3, Vector3), ImuError> {
        // Read 12 contiguous bytes: Gyro X, Y, Z (6 bytes) followed by Accel X, Y, Z (6 bytes)
        // using non-blocking hardware EasyDMA.
        // Use stack-allocated buffer for TX to ensure RAM address for EasyDMA.
        let mut buf = [0u8; 12];
        let reg = [OUTX_L_G_REG];
        with_timeout(
            Duration::from_millis(5),
            self.twim.write_read(LSM6DS3_ADDR, &reg, &mut buf),
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

/// Dedicated high-rate IMU worker task (Tier 1: Fast Sensor Bus).
///
/// Suspends in low-power WFE awaiting INT1 hardware data-ready interrupts @ 1666.6 Hz.
/// Performs 8:1 sinc boxcar decimation and publishes coherent 208.33 Hz frames to `IMU_WATCH`.
#[embassy_executor::task]
pub async fn imu_worker_task(
    mut imu: Lsm6ds3Driver<'static>,
    tx: embassy_sync::watch::Sender<'static, embassy_sync::blocking_mutex::raw::ThreadModeRawMutex, ImuDecimated, 2>,
) {
    defmt::info!("Starting Tier 1 IMU Worker Task (INT1 DRDY @ 1666.6 Hz)...");
    loop {
        match imu.next_frame().await {
            Ok(frame) => {
                tx.send(frame);
            }
            Err(e) => {
                defmt::error!("IMU read error in worker task: {:?}", e);
                Timer::after(Duration::from_millis(5)).await;
            }
        }
    }
}

