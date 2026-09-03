/// Arming and Failsafe Safety State Manager
/// Enforces physical safety interlocks to prevent unintended motor spin-up.

pub const THROTTLE_ZERO_THRESHOLD: u16 = 1050; // Throttle must be <= 1050us to arm
pub const ARM_SWITCH_THRESHOLD: u16 = 1800;    // Switch value > 1800 considered "ARM"

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmingStatus {
    Disarmed,
    ArmedAcro,
    ArmedAngle,
    RefusedThrottleRaised,
    RefusedSwitchArmedAtBoot,
    Failsafe,
}

#[derive(Debug, Clone, Copy)]
pub struct ArmingSafety {
    is_armed: bool,
    saw_disarmed_switch: bool,
}

impl Default for ArmingSafety {
    fn default() -> Self {
        Self::new()
    }
}

impl ArmingSafety {
    pub const fn new() -> Self {
        Self {
            is_armed: false,
            saw_disarmed_switch: false,
        }
    }

    pub fn reset(&mut self) {
        self.is_armed = false;
        self.saw_disarmed_switch = false;
    }

    /// Evaluates current RC switch and throttle inputs to determine safe arming state.
    ///
    /// * `arm_switch_value`: Microsecond value of Arm channel (typically CH5)
    /// * `mode_switch_value`: Microsecond value of Mode channel (CH6: <1500 = Angle, >=1500 = Acro)
    /// * `throttle_us`: Current throttle channel position in microseconds
    /// * `signal_lost`: True if RC packet timeout or link loss occurred
    pub fn update(
        &mut self,
        arm_switch_value: u16,
        mode_switch_value: u16,
        throttle_us: u16,
        signal_lost: bool,
    ) -> ArmingStatus {
        if signal_lost {
            self.is_armed = false;
            return ArmingStatus::Failsafe;
        }

        let switch_wants_arm = arm_switch_value > ARM_SWITCH_THRESHOLD;

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
                if throttle_us <= THROTTLE_ZERO_THRESHOLD {
                    self.is_armed = true;
                } else {
                    return ArmingStatus::RefusedThrottleRaised;
                }
            }

            // Successfully armed: choose Acro vs Angle mode
            if mode_switch_value < 1500 {
                ArmingStatus::ArmedAngle
            } else {
                ArmingStatus::ArmedAcro
            }
        } else {
            // Switch is in DISARM position
            self.is_armed = false;
            ArmingStatus::Disarmed
        }
    }

    #[inline(always)]
    pub fn is_armed(&self) -> bool {
        self.is_armed
    }
}
