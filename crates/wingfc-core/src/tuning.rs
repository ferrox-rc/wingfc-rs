//! In-flight dynamic PID parameter tuning via RC auxiliary channels.
//!
//! Enables pilots to tune rate PID gains (Pitch/Roll/Yaw Kp, Ki, Kd, Kff) in real-time
//! using transmitter rotary potentiometers, sliders, and bank selector switches.

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
    YawKp = 9,
    YawKi = 10,
    YawKd = 11,
    YawKff = 12,
}

/// Configuration for a single in-flight tuning channel slot.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TuneSlot {
    pub parameter: TuneParameter,
    pub channel_idx: u8, // 0-indexed (e.g. 5 for CH6, 6 for CH7)
    pub min_val: f32,
    pub max_val: f32,
}

impl TuneSlot {
    pub const DEFAULT: Self = Self {
        parameter: TuneParameter::None,
        channel_idx: 0,
        min_val: 0.0,
        max_val: 0.0,
    };

    pub const fn empty() -> Self {
        Self::DEFAULT
    }

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

    /// Applies tuning adjustment from RC channel input to pitch, roll, and yaw PID controllers.
    pub fn apply(
        &self,
        rc: &RcChannels,
        pitch_pid: &mut PidController,
        roll_pid: &mut PidController,
        yaw_pid: &mut PidController,
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
            TuneParameter::YawKp => yaw_pid.config.kp = val,
            TuneParameter::YawKi => yaw_pid.config.ki = val,
            TuneParameter::YawKd => yaw_pid.config.kd = val,
            TuneParameter::YawKff => yaw_pid.config.kff = val,
        }
    }
}

/// A bank of tuning slots activated when the selector channel falls within a pulse-width range.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TuneBank {
    pub range_min_us: u16,
    pub range_max_us: u16,
    pub slot_a: TuneSlot,
    pub slot_b: TuneSlot,
}

impl TuneBank {
    pub const DEFAULT: Self = Self {
        range_min_us: 0,
        range_max_us: 0,
        slot_a: TuneSlot::DEFAULT,
        slot_b: TuneSlot::DEFAULT,
    };

    pub const fn empty() -> Self {
        Self::DEFAULT
    }

    pub const fn new(
        range_min_us: u16,
        range_max_us: u16,
        slot_a: TuneSlot,
        slot_b: TuneSlot,
    ) -> Self {
        Self {
            range_min_us,
            range_max_us,
            slot_a,
            slot_b,
        }
    }

    /// Evaluates whether the selector channel pulse width falls within this bank's range.
    #[inline(always)]
    pub fn is_active(&self, selector_us: u16) -> bool {
        selector_us >= self.range_min_us && selector_us <= self.range_max_us
    }
}

/// Maximum number of selectable tuning banks supported.
pub const MAX_TUNE_BANKS: usize = 6;

/// In-flight dynamic PID parameter tuning supervisor holding multiple selectable banks.
#[derive(Debug, Clone, Copy, Default)]
pub struct InFlightTuning {
    pub selector_channel_idx: u8, // 0-indexed (e.g. 4 for CH5), or 0xFF if disabled/single-bank
    pub num_banks: u8,
    pub banks: [TuneBank; MAX_TUNE_BANKS],
}

impl InFlightTuning {
    pub const fn new(
        selector_channel_idx: u8,
        num_banks: u8,
        banks: [TuneBank; MAX_TUNE_BANKS],
    ) -> Self {
        Self {
            selector_channel_idx,
            num_banks,
            banks,
        }
    }

    /// Updates PID controllers based on active bank selected via RC auxiliary channels.
    pub fn update(
        &self,
        rc: &RcChannels,
        pitch_pid: &mut PidController,
        roll_pid: &mut PidController,
        yaw_pid: &mut PidController,
    ) {
        if self.num_banks == 0 {
            return;
        }

        let active_bank = if self.selector_channel_idx < 16 {
            let selector_us = rc.channels[self.selector_channel_idx as usize];
            if !(800..=2200).contains(&selector_us) {
                return;
            }
            let count = (self.num_banks as usize).min(MAX_TUNE_BANKS);
            let mut matched = None;
            let mut i = 0;
            while i < count {
                if self.banks[i].is_active(selector_us) {
                    matched = Some(&self.banks[i]);
                    break;
                }
                i += 1;
            }
            matched
        } else {
            // No selector channel: default to first bank
            Some(&self.banks[0])
        };

        if let Some(bank) = active_bank {
            bank.slot_a.apply(rc, pitch_pid, roll_pid, yaw_pid);
            bank.slot_b.apply(rc, pitch_pid, roll_pid, yaw_pid);
        }
    }
}
