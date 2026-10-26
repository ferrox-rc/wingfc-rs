//! Arming and Failsafe Safety State Manager
//! Enforces physical safety interlocks to prevent unintended motor spin-up.

use crate::mode::FlightMode;

pub const DEFAULT_THROTTLE_ZERO_THRESHOLD: u16 = 1050; // Throttle must be <= 1050us to arm
pub const DEFAULT_ARM_SWITCH_THRESHOLD: u16 = 1800; // Switch value > 1800 considered "ARM"

// Deprecated constant aliases retained for backwards compatibility
pub const THROTTLE_ZERO_THRESHOLD: u16 = DEFAULT_THROTTLE_ZERO_THRESHOLD;
pub const ARM_SWITCH_THRESHOLD: u16 = DEFAULT_ARM_SWITCH_THRESHOLD;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ArmingStatus {
    Disarmed,
    ArmedAcro,
    ArmedAngle,
    #[cfg(feature = "compass")]
    ArmedHeadingHold,
    #[cfg(feature = "baro")]
    ArmedAltHold,
    #[cfg(feature = "gps")]
    ArmedRth,
    RefusedThrottleRaised,
    RefusedSwitchArmedAtBoot,
    Failsafe,
}

impl ArmingStatus {
    /// Indicates whether the aircraft is currently armed in any operational flight mode.
    #[inline(always)]
    pub const fn is_armed(&self) -> bool {
        match self {
            ArmingStatus::ArmedAcro | ArmingStatus::ArmedAngle => true,
            #[cfg(feature = "compass")]
            ArmingStatus::ArmedHeadingHold => true,
            #[cfg(feature = "baro")]
            ArmingStatus::ArmedAltHold => true,
            #[cfg(feature = "gps")]
            ArmingStatus::ArmedRth => true,
            _ => false,
        }
    }

    /// Indicates whether the active flight mode relies on estimated Euler angles (pitch/roll/yaw)
    /// for closed-loop attitude stabilization.
    #[inline(always)]
    pub const fn requires_attitude(&self) -> bool {
        match self {
            ArmingStatus::ArmedAngle => true,
            #[cfg(feature = "compass")]
            ArmingStatus::ArmedHeadingHold => true,
            #[cfg(feature = "baro")]
            ArmingStatus::ArmedAltHold => true,
            #[cfg(feature = "gps")]
            ArmingStatus::ArmedRth => true,
            _ => false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ArmingSafety {
    is_armed: bool,
    saw_disarmed_switch: bool,
    failsafe_active: bool,
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
            failsafe_active: false,
            arm_switch_threshold,
            throttle_zero_threshold,
        }
    }

    pub fn reset(&mut self) {
        self.is_armed = false;
        self.saw_disarmed_switch = false;
        self.failsafe_active = false;
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
            if self.is_armed || self.failsafe_active {
                self.is_armed = false;
                self.failsafe_active = true;
                return ArmingStatus::Failsafe;
            } else {
                // On the ground while disarmed, loss of RC link simply prevents arming;
                // it should not trigger an emergency in-flight failsafe.
                return ArmingStatus::Disarmed;
            }
        }

        self.failsafe_active = false;

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
                #[cfg(feature = "compass")]
                FlightMode::HeadingHold => ArmingStatus::ArmedHeadingHold,
                #[cfg(feature = "baro")]
                FlightMode::AltHold => ArmingStatus::ArmedAltHold,
                #[cfg(feature = "gps")]
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
