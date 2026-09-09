//! Arming and Failsafe Safety State Manager
//! Enforces physical safety interlocks to prevent unintended motor spin-up.

use crate::mode::FlightMode;

pub const DEFAULT_THROTTLE_ZERO_THRESHOLD: u16 = 1050; // Throttle must be <= 1050us to arm
pub const DEFAULT_ARM_SWITCH_THRESHOLD: u16 = 1800; // Switch value > 1800 considered "ARM"

// Deprecated constant aliases retained for backwards compatibility
pub const THROTTLE_ZERO_THRESHOLD: u16 = DEFAULT_THROTTLE_ZERO_THRESHOLD;
pub const ARM_SWITCH_THRESHOLD: u16 = DEFAULT_ARM_SWITCH_THRESHOLD;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmingStatus {
    Disarmed,
    ArmedAcro,
    ArmedAngle,
    ArmedHeadingHold,
    ArmedAltHold,
    ArmedRth,
    RefusedThrottleRaised,
    RefusedSwitchArmedAtBoot,
    Failsafe,
}

#[derive(Debug, Clone, Copy)]
pub struct ArmingSafety {
    is_armed: bool,
    saw_disarmed_switch: bool,
    arm_switch_threshold: u16,
    throttle_zero_threshold: u16,
}

impl Default for ArmingSafety {
    fn default() -> Self {
        Self::new()
    }
}

impl ArmingSafety {
    /// Creates an arming safety manager with standard default thresholds (1800us arm, 1050us throttle).
    pub const fn new() -> Self {
        Self::with_thresholds(
            DEFAULT_ARM_SWITCH_THRESHOLD,
            DEFAULT_THROTTLE_ZERO_THRESHOLD,
        )
    }

    /// Creates an arming safety manager with custom user-configured thresholds.
    ///
    /// * `arm_switch_threshold`: Channel pulse (microseconds) above which arming is requested (e.g. 1800).
    /// * `throttle_zero_threshold`: Maximum pulse (microseconds) permitting transition to armed (e.g. 1050).
    pub const fn with_thresholds(arm_switch_threshold: u16, throttle_zero_threshold: u16) -> Self {
        Self {
            is_armed: false,
            saw_disarmed_switch: false,
            arm_switch_threshold,
            throttle_zero_threshold,
        }
    }

    pub fn reset(&mut self) {
        self.is_armed = false;
        self.saw_disarmed_switch = false;
    }

    /// Configured arm switch threshold in microseconds.
    pub const fn arm_switch_threshold(&self) -> u16 {
        self.arm_switch_threshold
    }

    /// Configured zero-throttle threshold in microseconds.
    pub const fn throttle_zero_threshold(&self) -> u16 {
        self.throttle_zero_threshold
    }

    /// Evaluates current RC switch, flight mode, and throttle inputs to determine safe arming state.
    ///
    /// * `arm_switch_value`: Microsecond value of Arm channel (typically CH5)
    /// * `active_mode`: Evaluated flight mode from INAV-style mode matrix
    /// * `throttle_us`: Current throttle channel position in microseconds
    /// * `signal_lost`: True if RC packet timeout or link loss occurred
    pub fn update_with_mode(
        &mut self,
        arm_switch_value: u16,
        active_mode: FlightMode,
        throttle_us: u16,
        signal_lost: bool,
    ) -> ArmingStatus {
        if signal_lost {
            self.is_armed = false;
            return ArmingStatus::Failsafe;
        }

        let switch_wants_arm = arm_switch_value > self.arm_switch_threshold;

        // Boot-arm interlock: Pilot must have switch in DISARMED state at least once
        if !switch_wants_arm {
            self.saw_disarmed_switch = true;
        }

        if switch_wants_arm {
            if !self.saw_disarmed_switch {
                // Transmitter was powered on with arm switch already HIGH
                self.is_armed = false;
                return ArmingStatus::RefusedSwitchArmedAtBoot;
            }

            if !self.is_armed {
                // Transitioning from Disarmed -> Armed requires zero throttle
                if throttle_us <= self.throttle_zero_threshold {
                    self.is_armed = true;
                } else {
                    return ArmingStatus::RefusedThrottleRaised;
                }
            }

            // Successfully armed: map active FlightMode to ArmingStatus
            match active_mode {
                FlightMode::Acro => ArmingStatus::ArmedAcro,
                FlightMode::Angle => ArmingStatus::ArmedAngle,
                FlightMode::HeadingHold => ArmingStatus::ArmedHeadingHold,
                FlightMode::AltHold => ArmingStatus::ArmedAltHold,
                FlightMode::Rth => ArmingStatus::ArmedRth,
            }
        } else {
            // Switch is in DISARM position
            self.is_armed = false;
            ArmingStatus::Disarmed
        }
    }

    /// Legacy convenience wrapper evaluating binary mode switch (<1500 = Angle, >=1500 = Acro).
    pub fn update(
        &mut self,
        arm_switch_value: u16,
        mode_switch_value: u16,
        throttle_us: u16,
        signal_lost: bool,
    ) -> ArmingStatus {
        let mode = if mode_switch_value < 1500 {
            FlightMode::Angle
        } else {
            FlightMode::Acro
        };
        self.update_with_mode(arm_switch_value, mode, throttle_us, signal_lost)
    }

    #[inline(always)]
    pub fn is_armed(&self) -> bool {
        self.is_armed
    }
}
