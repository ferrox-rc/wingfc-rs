//! INAV-style Flight Mode Selection Matrix and Cascaded Navigation Controllers.
//!
//! Provides multi-switch range-based flight mode evaluation and cascaded outer-loop
//! controllers for Heading Hold and Altitude Hold.

use crate::math::DEG_TO_RAD;
use core::f32::consts::PI;

/// Normalizes an angle in radians to [-PI, PI].
#[inline]
pub fn wrap_to_pi(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

/// Available flight modes in WingFC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlightMode {
    /// Pure rate control (acrobatic / manual rate)
    #[default]
    Acro,
    /// Attitude self-leveling (bank & pitch angle limits)
    Angle,
    /// Compass/gyro heading lock (cascaded outer loop onto Angle roll)
    HeadingHold,
    /// Barometer/vertical-KF altitude lock (cascaded outer loop onto Angle pitch)
    AltHold,
    /// Full autonomous Return-to-Home (GPS navigation)
    Rth,
}

/// A single mode activation range (INAV/Betaflight style).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeRange {
    pub mode: FlightMode,
    /// 0-indexed RC channel (e.g. 5 for CH6 / AUX2)
    pub channel_idx: usize,
    /// Minimum microsecond pulse width (inclusive, e.g. 1300)
    pub min_us: u16,
    /// Maximum microsecond pulse width (inclusive, e.g. 1700)
    pub max_us: u16,
}

impl ModeRange {
    pub const fn new(mode: FlightMode, channel_idx: usize, min_us: u16, max_us: u16) -> Self {
        Self {
            mode,
            channel_idx,
            min_us,
            max_us,
        }
    }

    #[inline(always)]
    pub fn is_active(&self, channel_us: u16) -> bool {
        channel_us >= self.min_us && channel_us <= self.max_us
    }
}

/// Mode selection manager evaluating an array of `ModeRange` rules.
///
/// Priority order is determined by rule order or specific mode precedence (RTH > AltHold > HeadingHold > Angle > Acro).
#[derive(Debug, Clone)]
pub struct ModeMatrix<const N: usize> {
    pub ranges: [ModeRange; N],
}

impl<const N: usize> ModeMatrix<N> {
    pub const fn new(ranges: [ModeRange; N]) -> Self {
        Self { ranges }
    }

    /// Evaluates current RC channel inputs and determines the active flight mode.
    ///
    /// If multiple modes match, highest safety/capability priority wins:
    /// `Rth` > `AltHold` > `HeadingHold` > `Angle` > `Acro`.
    pub fn evaluate(&self, channels: &[u16]) -> FlightMode {
        let mut active_mode = FlightMode::Acro;

        for range in &self.ranges {
            if range.channel_idx < channels.len() {
                let val = channels[range.channel_idx];
                if range.is_active(val) {
                    active_mode = Self::higher_priority(active_mode, range.mode);
                }
            }
        }

        active_mode
    }

    #[inline(always)]
    fn priority_score(mode: FlightMode) -> u8 {
        match mode {
            FlightMode::Acro => 0,
            FlightMode::Angle => 1,
            FlightMode::HeadingHold => 2,
            FlightMode::AltHold => 3,
            FlightMode::Rth => 4,
        }
    }

    #[inline(always)]
    fn higher_priority(a: FlightMode, b: FlightMode) -> FlightMode {
        if Self::priority_score(b) > Self::priority_score(a) {
            b
        } else {
            a
        }
    }
}

// =============================================================================
// Cascaded Heading Hold Controller
// =============================================================================

#[derive(Debug, Clone, Copy)]
pub struct HeadingHoldConfig {
    /// Proportional gain converting heading error (rad) to bank angle target (rad)
    pub kp_heading: f32,
    /// Maximum allowable bank angle in Heading Hold (rad)
    pub max_bank_angle_rad: f32,
    /// Pilot yaw stick deadband fraction to allow override (e.g. 0.05 = 5%)
    pub stick_deadband: f32,
}

impl Default for HeadingHoldConfig {
    fn default() -> Self {
        Self {
            kp_heading: 1.5,
            max_bank_angle_rad: 35.0 * DEG_TO_RAD,
            stick_deadband: 0.05,
        }
    }
}

pub struct HeadingHoldController {
    config: HeadingHoldConfig,
    target_heading_rad: f32,
    is_latched: bool,
}

impl Default for HeadingHoldController {
    fn default() -> Self {
        Self::new(HeadingHoldConfig::default())
    }
}

impl HeadingHoldController {
    pub const fn new(config: HeadingHoldConfig) -> Self {
        Self {
            config,
            target_heading_rad: 0.0,
            is_latched: false,
        }
    }

    pub fn reset(&mut self) {
        self.is_latched = false;
    }

    /// Update heading hold controller.
    ///
    /// * `current_heading_rad`: Current vehicle heading from AHRS (0..2pi or -pi..pi)
    /// * `pilot_roll_stick`: Pilot normalized roll stick in [-1.0, 1.0]
    /// * `pilot_yaw_stick`: Pilot normalized yaw stick in [-1.0, 1.0]
    ///
    /// Returns target roll angle in radians for Angle mode.
    pub fn update(
        &mut self,
        current_heading_rad: f32,
        pilot_roll_stick: f32,
        pilot_yaw_stick: f32,
    ) -> f32 {
        // If pilot gives significant roll or yaw stick command, relinquish hold and latch current heading
        if pilot_roll_stick.abs() > self.config.stick_deadband
            || pilot_yaw_stick.abs() > self.config.stick_deadband
            || !self.is_latched
        {
            self.target_heading_rad = current_heading_rad;
            self.is_latched = true;
            return pilot_roll_stick * self.config.max_bank_angle_rad;
        }

        // Calculate heading error wrapped to [-pi, pi]
        let error = wrap_to_pi(self.target_heading_rad - current_heading_rad);

        // Commanded bank angle proportional to heading error
        let bank_target = self.config.kp_heading * error;
        bank_target.clamp(
            -self.config.max_bank_angle_rad,
            self.config.max_bank_angle_rad,
        )
    }

    pub fn target_heading_rad(&self) -> f32 {
        self.target_heading_rad
    }
}

// =============================================================================
// Cascaded Altitude Hold Controller
// =============================================================================

#[derive(Debug, Clone, Copy)]
pub struct AltHoldConfig {
    /// Proportional gain converting altitude error (m) to target vertical speed (m/s)
    pub kp_alt: f32,
    /// Proportional gain converting vertical speed error (m/s) to target pitch angle (rad)
    pub kp_climb: f32,
    /// Maximum vertical climb/sink rate (m/s)
    pub max_climb_rate_ms: f32,
    /// Maximum pitch angle commanded during altitude hold (rad)
    pub max_pitch_rad: f32,
    /// Cruise throttle pulse in microseconds
    pub cruise_throttle_us: u16,
    /// Throttle stick deadband in microseconds around center (e.g. 1400..1600)
    pub throttle_deadband_us: u16,
}

impl Default for AltHoldConfig {
    fn default() -> Self {
        Self {
            kp_alt: 1.2,
            kp_climb: 0.15,
            max_climb_rate_ms: 3.5,
            max_pitch_rad: 25.0 * DEG_TO_RAD,
            cruise_throttle_us: 1500,
            throttle_deadband_us: 100,
        }
    }
}

pub struct AltHoldController {
    config: AltHoldConfig,
    target_altitude_m: f32,
    is_latched: bool,
}

impl Default for AltHoldController {
    fn default() -> Self {
        Self::new(AltHoldConfig::default())
    }
}

impl AltHoldController {
    pub const fn new(config: AltHoldConfig) -> Self {
        Self {
            config,
            target_altitude_m: 0.0,
            is_latched: false,
        }
    }

    pub fn reset(&mut self) {
        self.is_latched = false;
    }

    /// Update altitude hold controller.
    ///
    /// * `current_altitude_m`: Current altitude from Barometer / Vertical KF
    /// * `vertical_velocity_ms`: Current vertical climb rate in m/s
    /// * `pilot_pitch_stick`: Pilot normalized pitch stick in [-1.0, 1.0]
    /// * `pilot_throttle_us`: Pilot raw throttle stick in microseconds
    ///
    /// Returns: `(target_pitch_rad, output_throttle_us)`
    pub fn update(
        &mut self,
        current_altitude_m: f32,
        vertical_velocity_ms: f32,
        pilot_pitch_stick: f32,
        pilot_throttle_us: u16,
    ) -> (f32, u16) {
        // Pilot can command climb/descent via pitch stick or throttle stick
        let throttle_centered = (pilot_throttle_us as i32 - self.config.cruise_throttle_us as i32)
            .unsigned_abs() as u16
            <= self.config.throttle_deadband_us;

        if pilot_pitch_stick.abs() > 0.08 || !throttle_centered || !self.is_latched {
            self.target_altitude_m = current_altitude_m;
            self.is_latched = true;
            let commanded_pitch = pilot_pitch_stick * self.config.max_pitch_rad;
            return (commanded_pitch, pilot_throttle_us);
        }

        // Altitude error: target - current
        let alt_error = self.target_altitude_m - current_altitude_m;

        // Outer P-loop: Altitude error -> Target vertical velocity
        let target_vz = (self.config.kp_alt * alt_error).clamp(
            -self.config.max_climb_rate_ms,
            self.config.max_climb_rate_ms,
        );

        // Inner P-loop: Vertical velocity error -> Commanded pitch angle
        let vz_error = target_vz - vertical_velocity_ms;
        let pitch_target = (self.config.kp_climb * vz_error)
            .clamp(-self.config.max_pitch_rad, self.config.max_pitch_rad);

        (pitch_target, self.config.cruise_throttle_us)
    }

    pub fn target_altitude_m(&self) -> f32 {
        self.target_altitude_m
    }
}
