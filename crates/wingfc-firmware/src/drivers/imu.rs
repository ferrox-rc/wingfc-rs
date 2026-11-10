use crate::config::{
    BOARD_ORIENTATION, IMU_DECIMATION, IMU_ODR_G_REG_VAL, IMU_ODR_XL_REG_VAL, NOMINAL_DT,
    NOMINAL_LOOP_HZ,
};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use wingfc_core::downsample::ImuDecimated;
pub use wingfc_drivers::imu::{ImuError, Lsm6ds3Config, Lsm6ds3Driver};

/// Returns the static [`Lsm6ds3Config`] generated from `wingfc.yaml`.
pub const fn board_imu_config() -> Lsm6ds3Config {
    Lsm6ds3Config {
        orientation: BOARD_ORIENTATION,
        odr_xl_reg_val: IMU_ODR_XL_REG_VAL,
        odr_g_reg_val: IMU_ODR_G_REG_VAL,
        decimation: IMU_DECIMATION,
        nominal_dt: NOMINAL_DT,
        nominal_loop_hz: NOMINAL_LOOP_HZ,
    }
}

/// Dedicated high-rate IMU worker task (Tier 1: Fast Sensor Bus).
///
/// Suspends in low-power WFE awaiting INT1 hardware data-ready interrupts @ 1666.6 Hz.
/// Performs 8:1 sinc boxcar decimation and publishes coherent 208.33 Hz frames to `IMU_WATCH`.
#[embassy_executor::task]
pub async fn imu_worker_task(
    mut imu: Lsm6ds3Driver<embassy_nrf::twim::Twim<'static>, embassy_nrf::gpio::Input<'static>>,
    tx: embassy_sync::watch::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
        ImuDecimated,
        2,
    >,
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

/// Helper constructor binding `Lsm6ds3Driver` with the active aircraft `wingfc.yaml` config.
pub fn new_board_imu<I2C: I2c, INT: Wait>(i2c: I2C, int1: INT) -> Lsm6ds3Driver<I2C, INT> {
    Lsm6ds3Driver::new(i2c, int1, board_imu_config())
}
