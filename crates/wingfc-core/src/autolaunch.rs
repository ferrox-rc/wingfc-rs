use crate::math::{clamp, map_range, DEG_TO_RAD};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AutolaunchState {
    Inactive,
    WaitingForThrow,
    MotorDelay,
    Climbing,
    Completed,
}

#[derive(Debug, Clone, Copy)]
pub struct AutolaunchConfig {
    pub throw_accel_threshold_ms2: f32, // Forward acceleration threshold in m/s^2 (e.g. 14.0 m/s^2 ~ 1.43g)
    pub throw_detect_time: f32,         // Duration accel must be sustained (e.g. 0.06s)
    pub motor_delay_time: f32,          // Finger safety delay before motor spinup (e.g. 0.20s)
    pub throttle_ramp_time: f32,        // Ramp up duration to prevent torque roll (e.g. 0.40s)
    pub climb_throttle_us: u16,         // Target climb throttle in microseconds (e.g. 1750us)
    pub climb_pitch_rad: f32,           // Target climb pitch angle in radians (e.g. 15 deg)
    pub stick_abort_threshold: f32, // Stick deflection threshold to cancel autolaunch (e.g. 0.20)
    pub max_climb_duration: f32,    // Maximum autolaunch duration before auto-exit (e.g. 5.0s)
}

impl Default for AutolaunchConfig {
    fn default() -> Self {
        Self {
            throw_accel_threshold_ms2: 13.5,
            throw_detect_time: 0.05,
            motor_delay_time: 0.20,
            throttle_ramp_time: 0.40,
            climb_throttle_us: 1750,
            climb_pitch_rad: 15.0 * DEG_TO_RAD,
            stick_abort_threshold: 0.20,
            max_climb_duration: 5.0,
        }
    }
}

pub struct AutolaunchController {
    pub config: AutolaunchConfig,
    state: AutolaunchState,
    accel_timer: f32,
    state_timer: f32,
    current_throttle_us: u16,
}

impl Default for AutolaunchController {
    fn default() -> Self {
        Self::new(AutolaunchConfig::default())
    }
}

impl AutolaunchController {
    pub const fn new(config: AutolaunchConfig) -> Self {
        Self {
            config,
            state: AutolaunchState::Inactive,
            accel_timer: 0.0,
            state_timer: 0.0,
            current_throttle_us: 1000,
        }
    }

    /// Arm and prime autolaunch mode
    pub fn prime(&mut self) {
        self.state = AutolaunchState::WaitingForThrow;
        self.accel_timer = 0.0;
        self.state_timer = 0.0;
        self.current_throttle_us = 1000;
    }

    /// Abort autolaunch and return to manual flight
    pub fn abort(&mut self) {
        self.state = AutolaunchState::Completed;
        self.current_throttle_us = 1000;
    }

    #[inline(always)]
    pub fn state(&self) -> AutolaunchState {
        self.state
    }

    #[inline(always)]
    pub fn is_active(&self) -> bool {
        matches!(
            self.state,
            AutolaunchState::WaitingForThrow
                | AutolaunchState::MotorDelay
                | AutolaunchState::Climbing
        )
    }

    /// Update autolaunch controller for one flight cycle
    ///
    /// * `forward_accel_ms2`: Accelerometer X axis in m/s^2
    /// * `pitch_stick`: Pitch stick deflection in [-1.0, 1.0]
    /// * `roll_stick`: Roll stick deflection in [-1.0, 1.0]
    /// * `dt`: Delta time in seconds (e.g. 0.002 for 500Hz)
    ///
    /// Returns: `(target_pitch_mix, target_roll_mix, motor_throttle_us)`
    pub fn update(
        &mut self,
        forward_accel_ms2: f32,
        pitch_stick: f32,
        roll_stick: f32,
        dt: f32,
    ) -> (Option<f32>, Option<f32>, u16) {
        // Stick abort check: Any significant pilot stick input cancels autolaunch immediately
        if self.is_active()
            && (pitch_stick.abs() > self.config.stick_abort_threshold
                || roll_stick.abs() > self.config.stick_abort_threshold)
        {
            self.abort();
        }

        match self.state {
            AutolaunchState::Inactive | AutolaunchState::Completed => {
                // Pass control to normal flight loop
                (None, None, 1000)
            }
            AutolaunchState::WaitingForThrow => {
                // Keep motor stopped while waiting in pilot hand
                self.current_throttle_us = 1000;

                if forward_accel_ms2 >= self.config.throw_accel_threshold_ms2 {
                    self.accel_timer += dt;
                    if self.accel_timer >= self.config.throw_detect_time {
                        // Throw verified: start motor delay timer
                        self.state = AutolaunchState::MotorDelay;
                        self.state_timer = 0.0;
                    }
                } else {
                    self.accel_timer = 0.0;
                }

                // Hold level wings and gentle climb angle on surfaces during throw
                (Some(0.25), Some(0.0), 1000)
            }
            AutolaunchState::MotorDelay => {
                // Finger safety delay: aircraft is traveling forward away from hands
                self.state_timer += dt;
                self.current_throttle_us = 1000;

                if self.state_timer >= self.config.motor_delay_time {
                    self.state = AutolaunchState::Climbing;
                    self.state_timer = 0.0;
                }

                (Some(0.25), Some(0.0), 1000)
            }
            AutolaunchState::Climbing => {
                self.state_timer += dt;

                // Smoothly ramp throttle to climb throttle
                let ramp_fraction =
                    clamp(self.state_timer / self.config.throttle_ramp_time, 0.0, 1.0);
                self.current_throttle_us = map_range(
                    ramp_fraction,
                    0.0,
                    1.0,
                    1000.0,
                    self.config.climb_throttle_us as f32,
                ) as u16;

                // Timeout check: Exit autolaunch after max climb duration
                if self.state_timer >= self.config.max_climb_duration {
                    self.state = AutolaunchState::Completed;
                }

                // Hold commanded climb pitch and 0 roll (wings level)
                (Some(0.30), Some(0.0), self.current_throttle_us)
            }
        }
    }
}
