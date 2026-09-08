//! In-flight dynamic PID parameter tuning via RC auxiliary channels.
//!
//! Enables pilots to tune rate PID gains (Pitch/Roll Kp, Ki, Kd, Kff) in real-time
//! using transmitter rotary potentiometers or sliders assigned to RC channels.

use crate::math::map_range;
use crate::pid::PidController;
use crate::protocol::RcChannels;

/// Selectable parameter for in-flight RC tuning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum TuneParameter {
    #[default]
    None = 0,
    PitchKp = 1,
    RollKp = 2,
    PitchKi = 3,
    RollKi = 4,
    PitchKd = 5,
    RollKd = 6,
    PitchKff = 7,
    RollKff = 8,
}

/// Configuration for a single in-flight tuning channel slot.
#[derive(Debug, Clone, Copy, Default)]
pub struct TuneSlot {
    pub parameter: TuneParameter,
    pub channel_idx: u8, // 0-indexed (e.g. 5 for CH6, 6 for CH7)
    pub min_val: f32,
    pub max_val: f32,
}

impl TuneSlot {
    pub const fn new(
        parameter: TuneParameter,
        channel_idx: u8,
        min_val: f32,
        max_val: f32,
    ) -> Self {
        Self {
            parameter,
            channel_idx,
            min_val,
            max_val,
        }
    }

    /// Applies tuning adjustment from RC channel input to pitch and roll PID controllers.
    pub fn apply(
        &self,
        rc: &RcChannels,
        pitch_pid: &mut PidController,
        roll_pid: &mut PidController,
    ) {
        if self.parameter == TuneParameter::None || self.channel_idx >= 16 {
            return;
        }

        let raw_us = rc.channels[self.channel_idx as usize];
        if !(800..=2200).contains(&raw_us) {
            return;
        }

        let val = map_range(raw_us as f32, 1000.0, 2000.0, self.min_val, self.max_val);

        match self.parameter {
            TuneParameter::None => {}
            TuneParameter::PitchKp => pitch_pid.config.kp = val,
            TuneParameter::RollKp => roll_pid.config.kp = val,
            TuneParameter::PitchKi => pitch_pid.config.ki = val,
            TuneParameter::RollKi => roll_pid.config.ki = val,
            TuneParameter::PitchKd => pitch_pid.config.kd = val,
            TuneParameter::RollKd => roll_pid.config.kd = val,
            TuneParameter::PitchKff => pitch_pid.config.kff = val,
            TuneParameter::RollKff => roll_pid.config.kff = val,
        }
    }
}

/// In-flight potentiometer tuning supervisor holding slots A and B.
#[derive(Debug, Clone, Copy, Default)]
pub struct InFlightTuning {
    pub slot_a: TuneSlot,
    pub slot_b: TuneSlot,
}

impl InFlightTuning {
    pub const fn new(slot_a: TuneSlot, slot_b: TuneSlot) -> Self {
        Self { slot_a, slot_b }
    }

    pub fn update(
        &self,
        rc: &RcChannels,
        pitch_pid: &mut PidController,
        roll_pid: &mut PidController,
    ) {
        self.slot_a.apply(rc, pitch_pid, roll_pid);
        self.slot_b.apply(rc, pitch_pid, roll_pid);
    }
}
