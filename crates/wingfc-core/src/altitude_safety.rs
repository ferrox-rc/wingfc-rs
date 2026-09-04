/// Altitude Safety & Flight Envelope Protection Controller
/// Enforces Ground Collision Avoidance ("Hard Deck"), Legal Airspace Ceiling, and Stall Detection.

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AltitudeIntervention {
    None,
    HardDeckPullUp,
    CeilingPitchDown,
    StallRecovery,
}

#[derive(Debug, Clone, Copy)]
pub struct AltitudeSafetyConfig {
    pub hard_deck_altitude_m: f32, // Minimum floor altitude AGL (e.g. 15.0 meters)
    pub legal_ceiling_m: f32,      // Maximum legal ceiling AGL (e.g. 120.0 meters / 400 ft)
    pub stall_sink_rate_ms: f32,   // Vertical sink rate threshold for stall detect (e.g. -3.5 m/s)
    pub hard_deck_pull_pitch: f32, // Commanded pitch mix during Hard Deck pull-up (e.g. +0.45)
    pub stall_recovery_pitch: f32, // Commanded pitch mix during Stall Recovery (e.g. -0.25 pitch down)
}

impl Default for AltitudeSafetyConfig {
    fn default() -> Self {
        Self {
            hard_deck_altitude_m: 15.0,
            legal_ceiling_m: 120.0,
            stall_sink_rate_ms: -3.5,
            hard_deck_pull_pitch: 0.45,
            stall_recovery_pitch: -0.25,
        }
    }
}

pub struct AltitudeSafetyController {
    pub config: AltitudeSafetyConfig,
}

impl Default for AltitudeSafetyController {
    fn default() -> Self {
        Self::new(AltitudeSafetyConfig::default())
    }
}

impl AltitudeSafetyController {
    pub const fn new(config: AltitudeSafetyConfig) -> Self {
        Self { config }
    }

    /// Evaluates altitude and vertical velocity to determine if safety envelope intervention is required.
    ///
    /// * `altitude_agl_m`: Current estimated altitude Above Ground Level in meters
    /// * `vertical_velocity_ms`: Vertical climb/sink rate in m/s (positive = climbing, negative = sinking)
    /// * `pitch_angle_rad`: Current aircraft pitch angle in radians
    /// * `pilot_pitch_cmd`: Pilot's normalized pitch stick input in [-1.0, 1.0]
    /// * `pilot_throttle_us`: Pilot's throttle input in microseconds
    ///
    /// Returns: `(intervention_type, final_pitch_mix, final_throttle_us)`
    pub fn evaluate(
        &self,
        altitude_agl_m: f32,
        vertical_velocity_ms: f32,
        pitch_angle_rad: f32,
        pilot_pitch_cmd: f32,
        pilot_throttle_us: u16,
    ) -> (AltitudeIntervention, f32, u16) {
        // 1. Stall / Freefall Detection:
        // Aircraft is rapidly falling (sink rate < threshold) despite pitch being level or nose up
        if vertical_velocity_ms < self.config.stall_sink_rate_ms && pitch_angle_rad > -0.05 {
            return (
                AltitudeIntervention::StallRecovery,
                self.config.stall_recovery_pitch, // Drop nose to regain airspeed
                1900,                             // Apply full power
            );
        }

        // 2. "Hard Deck" Ground Collision Floor Override:
        // Aircraft is below minimum floor and descending
        if altitude_agl_m < self.config.hard_deck_altitude_m && vertical_velocity_ms < -0.5 {
            // Override pilot pitch to pull nose up and ensure flying throttle
            let safe_throttle = pilot_throttle_us.max(1650);
            return (
                AltitudeIntervention::HardDeckPullUp,
                self.config.hard_deck_pull_pitch,
                safe_throttle,
            );
        }

        // 3. Legal 400ft Airspace Ceiling Enforcement:
        // Aircraft is approaching or exceeding 120m ceiling and climbing
        if altitude_agl_m >= self.config.legal_ceiling_m && vertical_velocity_ms > 0.5 {
            let gentle_push_down = pilot_pitch_cmd.min(-0.20);
            let capped_throttle = pilot_throttle_us.min(1450);
            return (
                AltitudeIntervention::CeilingPitchDown,
                gentle_push_down,
                capped_throttle,
            );
        }

        // Normal flight: No intervention needed
        (
            AltitudeIntervention::None,
            pilot_pitch_cmd,
            pilot_throttle_us,
        )
    }
}
