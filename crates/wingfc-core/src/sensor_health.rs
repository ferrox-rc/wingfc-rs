//! Sensor health monitoring state machine for navigation and stabilization sensors.
//!
//! Provides timeout detection and plausibility checking for Barometer, Compass, and GPS.

/// Health state of a physical flight sensor or subsystem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum SensorHealth {
    /// Sensor is not installed, not detected at boot, or explicitly disabled in configuration.
    #[default]
    Absent,
    /// Sensor is functioning normally and providing fresh, valid telemetry.
    Healthy,
    /// Sensor has timed out (lost samples) or reported out-of-bounds anomalous data.
    Degraded,
}

impl SensorHealth {
    /// Returns true if the sensor is present and reporting healthy data.
    #[inline]
    pub const fn is_healthy(&self) -> bool {
        matches!(self, Self::Healthy)
    }

    /// Returns true if the sensor is installed/configured (regardless of health status).
    #[inline]
    pub const fn is_present(&self) -> bool {
        !matches!(self, Self::Absent)
    }
}

/// Aggregated system sensor health status container.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct SystemSensorHealth {
    pub baro: SensorHealth,
    pub compass: SensorHealth,
    pub gps: SensorHealth,
}

impl SystemSensorHealth {
    pub const fn new() -> Self {
        Self {
            baro: SensorHealth::Absent,
            compass: SensorHealth::Absent,
            gps: SensorHealth::Absent,
        }
    }

    /// Returns true if all configured/present sensors are healthy.
    pub fn all_present_healthy(&self) -> bool {
        let baro_ok = !self.baro.is_present() || self.baro.is_healthy();
        let compass_ok = !self.compass.is_present() || self.compass.is_healthy();
        let gps_ok = !self.gps.is_present() || self.gps.is_healthy();
        baro_ok && compass_ok && gps_ok
    }
}

/// Plausibility bounds for earth magnetic field magnitude in microteslas (uT) or raw scaled units.
/// Typical Earth magnetic field ranges between 25 uT and 65 uT.
pub const MAG_MIN_PLAUSIBLE_UT: f32 = 10.0;
pub const MAG_MAX_PLAUSIBLE_UT: f32 = 90.0;

/// Checks if a measured magnetic vector magnitude is physically plausible for Earth field.
#[inline]
pub fn is_mag_magnitude_plausible(mag: crate::math::Vector3) -> bool {
    let mag_sq = mag.x * mag.x + mag.y * mag.y + mag.z * mag.z;
    mag_sq >= (MAG_MIN_PLAUSIBLE_UT * MAG_MIN_PLAUSIBLE_UT)
        && mag_sq <= (MAG_MAX_PLAUSIBLE_UT * MAG_MAX_PLAUSIBLE_UT)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vector3;

    #[test]
    fn test_sensor_health_predicates() {
        assert!(!SensorHealth::Absent.is_present());
        assert!(!SensorHealth::Absent.is_healthy());

        assert!(SensorHealth::Healthy.is_present());
        assert!(SensorHealth::Healthy.is_healthy());

        assert!(SensorHealth::Degraded.is_present());
        assert!(!SensorHealth::Degraded.is_healthy());
    }

    #[test]
    fn test_system_sensor_health_aggregation() {
        let mut sys = SystemSensorHealth::new();
        // All absent -> healthy overall
        assert!(sys.all_present_healthy());

        sys.baro = SensorHealth::Healthy;
        assert!(sys.all_present_healthy());

        sys.baro = SensorHealth::Degraded;
        assert!(!sys.all_present_healthy());

        sys.baro = SensorHealth::Healthy;
        sys.gps = SensorHealth::Degraded;
        assert!(!sys.all_present_healthy());
    }

    #[test]
    fn test_mag_magnitude_plausibility() {
        // Normal field ~ 50 uT
        assert!(is_mag_magnitude_plausible(Vector3::new(30.0, 10.0, 38.7)));
        // Zero / dead sensor
        assert!(!is_mag_magnitude_plausible(Vector3::ZERO));
        // Extreme saturation near motor/ESC magnets (e.g. 150 uT)
        assert!(!is_mag_magnitude_plausible(Vector3::new(100.0, 100.0, 50.0)));
    }
}
