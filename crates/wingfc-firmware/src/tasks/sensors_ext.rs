//! External I2C Sensor Task (TWISPI1).
//!
//! Orchestrates asynchronous polling of external barometers (DPS310 / BMP280)
//! and magnetometers (QMC5883L / HMC5883L) sharing TWISPI1 (D4 / D5).
//! Broadcasts timestamped sensor snapshots via an Embassy Watch channel to the flight loop.

use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Timer};
use wingfc_core::math::Vector3;

#[cfg(feature = "baro")]
use crate::config::{BARO_ENABLED, BARO_I2C_ADDR, BARO_MODEL};
#[cfg(feature = "baro")]
use crate::drivers::baro::{pressure_to_altitude, BaroModel, Bmp280Driver, Dps310Driver};

#[cfg(feature = "compass")]
use crate::config::{COMPASS_ENABLED, COMPASS_MODEL};
#[cfg(feature = "compass")]
use crate::drivers::compass::{CompassModel, Hmc5883lDriver, Qmc5883lDriver};

/// External (slow) sensor poll period: 20 Hz navigation tier.
const EXT_SENSOR_PERIOD_MS: u64 = 50;

#[derive(Debug, Clone, Copy, Default)]
pub struct ExternalSensorSnapshot {
    pub baro_alt_m: f32,
    pub baro_temp_c: f32,
    pub baro_pressure_pa: f32,
    pub baro_updated: bool,
    pub mag: Vector3,
    pub mag_updated: bool,
}

#[embassy_executor::task]
pub async fn external_sensors_task(
    mut twim: Twim<'static>,
    sender: Sender<'static, ThreadModeRawMutex, ExternalSensorSnapshot, 2>,
) {
    defmt::info!("Starting External I2C Sensors Task on TWISPI1 (20 Hz)...");

    // 1. Initialize Barometer if enabled
    #[cfg(feature = "baro")]
    let mut dps310 = if BARO_ENABLED && BARO_MODEL == BaroModel::Dps310 {
        let mut d = Dps310Driver::new(BARO_I2C_ADDR);
        if let Err(e) = d.init(&mut twim).await {
            defmt::warn!("Failed to initialize DPS310 Barometer: {:?}", e);
        }
        Some(d)
    } else {
        None
    };

    #[cfg(feature = "baro")]
    let mut bmp280 = if BARO_ENABLED && BARO_MODEL == BaroModel::Bmp280 {
        let mut b = Bmp280Driver::new(BARO_I2C_ADDR);
        if let Err(e) = b.init(&mut twim).await {
            defmt::warn!("Failed to initialize BMP280 Barometer: {:?}", e);
        }
        Some(b)
    } else {
        None
    };

    // 2. Initialize Compass if enabled
    #[cfg(feature = "compass")]
    let mut qmc = if COMPASS_ENABLED && COMPASS_MODEL == CompassModel::Qmc5883l {
        let mut q = Qmc5883lDriver::new();
        if let Err(e) = q.init(&mut twim).await {
            defmt::warn!("Failed to initialize QMC5883L Compass: {:?}", e);
        }
        Some(q)
    } else {
        None
    };

    #[cfg(feature = "compass")]
    let mut hmc = if COMPASS_ENABLED && COMPASS_MODEL == CompassModel::Hmc5883l {
        let mut h = Hmc5883lDriver::new();
        if let Err(e) = h.init(&mut twim).await {
            defmt::warn!("Failed to initialize HMC5883L Compass: {:?}", e);
        }
        Some(h)
    } else {
        None
    };

    // 3. Tare initial ground pressure for relative altitude calculation
    #[cfg(feature = "baro")]
    let mut ground_pressure_pa = 101325.0f32;
    #[cfg(feature = "baro")]
    {
        if BARO_ENABLED {
            Timer::after(Duration::from_millis(50)).await;
            let mut sum_p = 0.0f32;
            let mut samples = 0;
            for _ in 0..15 {
                let p_res = if let Some(ref mut d) = dps310 {
                    d.read(&mut twim).await.map(|(p, _)| p).ok()
                } else if let Some(ref mut b) = bmp280 {
                    b.read(&mut twim).await.map(|(p, _)| p).ok()
                } else {
                    None
                };

                if let Some(p) = p_res {
                    if p > 30000.0 && p < 120000.0 {
                        sum_p += p;
                        samples += 1;
                    }
                }
                Timer::after(Duration::from_millis(20)).await;
            }

            if samples > 0 {
                ground_pressure_pa = sum_p / (samples as f32);
                defmt::info!("Barometer calibrated: Ground Tare Pressure = {} Pa", ground_pressure_pa);
            }
        }
    }

    let mut snapshot = ExternalSensorSnapshot::default();

    loop {
        Timer::after(Duration::from_millis(EXT_SENSOR_PERIOD_MS)).await; // 20 Hz loop

        snapshot.baro_updated = false;
        snapshot.mag_updated = false;

        // Poll Barometer
        #[cfg(feature = "baro")]
        {
            if BARO_ENABLED {
                let baro_res = if let Some(ref mut d) = dps310 {
                    d.read(&mut twim).await.ok()
                } else if let Some(ref mut b) = bmp280 {
                    b.read(&mut twim).await.ok()
                } else {
                    None
                };

                if let Some((press_pa, temp_c)) = baro_res {
                    snapshot.baro_pressure_pa = press_pa;
                    snapshot.baro_temp_c = temp_c;
                    snapshot.baro_alt_m = pressure_to_altitude(press_pa, ground_pressure_pa);
                    snapshot.baro_updated = true;
                }
            }
        }

        // Poll Compass
        #[cfg(feature = "compass")]
        {
            if COMPASS_ENABLED {
                let mag_res = if let Some(ref mut q) = qmc {
                    q.read(&mut twim).await.ok()
                } else if let Some(ref mut h) = hmc {
                    h.read(&mut twim).await.ok()
                } else {
                    None
                };

                if let Some(mag_vec) = mag_res {
                    snapshot.mag = mag_vec;
                    snapshot.mag_updated = true;
                }
            }
        }

        sender.send(snapshot);
    }
}
