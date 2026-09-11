use crate::math::clamp;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AirframeType {
    #[default]
    Elevon, // Traditional Flying Wing (Left & Right elevons + optional rudder)
    ElevonWithDiffThrust, // Flying Wing with Twin Motors for Yaw authority
    SingleAileronTTail,   // S1: Aileron, S2: Elevator, S4: Rudder
    DualAileronTTail,     // S1: Left Aileron, S2: Elevator, S4: Rudder, S5: Right Aileron
    SingleAileronVTail,   // S1: Aileron, S2: Right V-Tail, S4: Left V-Tail
    DualAileronVTail,     // S1: Left Aileron, S2: Right V-Tail, S4: Left V-Tail, S5: Right Aileron
}

#[derive(Debug, Clone, Copy)]
pub struct ServoConfig {
    pub min_us: u16,
    pub max_us: u16,
    pub subtrim_us: i16,
    pub reversed: bool,
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self {
            min_us: 1000,
            max_us: 2000,
            subtrim_us: 0,
            reversed: false,
        }
    }
}

impl ServoConfig {
    /// Evaluates the servo output pulse in microseconds for a normalized input command in `[-1.0, 1.0]`.
    ///
    /// Computes output via single-cycle multiply-add arithmetic (`VFMA` on Cortex-M4F),
    /// avoiding runtime division and branches.
    #[inline(always)]
    pub fn apply(&self, normalized: f32) -> u16 {
        let center = (self.min_us as f32 + self.max_us as f32) * 0.5;
        let half_span = (self.max_us as f32 - self.min_us as f32) * 0.5;
        let dir = if self.reversed {
            -normalized
        } else {
            normalized
        };
        let pulse = center + dir * half_span + self.subtrim_us as f32;
        clamp(pulse, self.min_us as f32, self.max_us as f32) as u16
    }

    /// Evaluates the neutral servo pulse width (normalized command = 0.0).
    #[inline(always)]
    pub fn neutral(&self) -> u16 {
        self.apply(0.0)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AirframeOutputs {
    pub servo1_us: u16,       // Primary Servo 1 (e.g. Left Elevon / Aileron)
    pub servo2_us: u16,       // Primary Servo 2 (e.g. Right Elevon / Elevator / Right V-Tail)
    pub servo3_us: u16,       // Primary Servo 3 (e.g. Rudder / Left V-Tail / Aux)
    pub servo4_us: u16,       // Primary Servo 4 (e.g. Right Aileron / Flap / Aux)
    pub servo5_us: u16,       // Expansion Servo 5 (Sense Plus PWM2 Ch 0)
    pub servo6_us: u16,       // Expansion Servo 6 (Sense Plus PWM2 Ch 1)
    pub servo7_us: u16,       // Expansion Servo 7 (Sense Plus PWM2 Ch 2)
    pub servo8_us: u16,       // Expansion Servo 8 (Sense Plus PWM2 Ch 3)
    pub motor1_throttle: u16, // 1000..2000 us (or DShot 0..2047)
    pub motor2_throttle: u16, // Twin motor if configured
}

pub struct AirframeMixer {
    pub airframe_type: AirframeType,
    pub servos: [ServoConfig; 8], // index 0..7 corresponds to Servo 1..8
    pub diff_thrust_weight: f32,  // 0.0 .. 1.0 fraction of throttle range (1000us)
}

impl AirframeMixer {
    pub fn new(airframe_type: AirframeType) -> Self {
        Self {
            airframe_type,
            servos: [ServoConfig::default(); 8],
            diff_thrust_weight: 0.15,
        }
    }

    pub fn with_diff_thrust(airframe_type: AirframeType, diff_thrust_weight: f32) -> Self {
        Self {
            airframe_type,
            servos: [ServoConfig::default(); 8],
            diff_thrust_weight,
        }
    }

    /// Mixes normalized pitch, roll, yaw commands in [-1.0, 1.0] and throttle in [1000, 2000].
    /// Only evaluates neutral pulses for channels unused by the active airframe type.
    pub fn mix(&self, pitch: f32, roll: f32, yaw: f32, throttle_us: u16) -> AirframeOutputs {
        let mut outputs = AirframeOutputs {
            servo5_us: self.servos[4].neutral(),
            servo6_us: self.servos[5].neutral(),
            servo7_us: self.servos[6].neutral(),
            servo8_us: self.servos[7].neutral(),
            ..AirframeOutputs::default()
        };

        match self.airframe_type {
            AirframeType::Elevon => {
                // Elevon mixing:
                // S1 (Left Elevon)  =  0.5 * Pitch + 0.5 * Roll
                // S2 (Right Elevon) = -0.5 * Pitch + 0.5 * Roll
                // S3 (Optional Rudder / Aux) = Yaw
                let s1_norm = clamp(0.5 * pitch + 0.5 * roll, -1.0, 1.0);
                let s2_norm = clamp(-0.5 * pitch + 0.5 * roll, -1.0, 1.0);

                outputs.servo1_us = self.servos[0].apply(s1_norm);
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo3_us = self.servos[2].apply(clamp(yaw, -1.0, 1.0));
                outputs.servo4_us = self.servos[3].neutral();

                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::ElevonWithDiffThrust => {
                let s1_norm = clamp(0.5 * pitch + 0.5 * roll, -1.0, 1.0);
                let s2_norm = clamp(-0.5 * pitch + 0.5 * roll, -1.0, 1.0);

                outputs.servo1_us = self.servos[0].apply(s1_norm);
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo3_us = self.servos[2].neutral();
                outputs.servo4_us = self.servos[3].neutral();

                let diff = (yaw * self.diff_thrust_weight * 1000.0) as i16;
                outputs.motor1_throttle =
                    clamp((throttle_us as i16 - diff) as f32, 1000.0, 2000.0) as u16;
                outputs.motor2_throttle =
                    clamp((throttle_us as i16 + diff) as f32, 1000.0, 2000.0) as u16;
            }
            AirframeType::SingleAileronTTail => {
                // S1: Aileron, S2: Elevator, S3: Rudder
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(clamp(pitch, -1.0, 1.0));
                outputs.servo3_us = self.servos[2].apply(clamp(yaw, -1.0, 1.0));
                outputs.servo4_us = self.servos[3].neutral();
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::DualAileronTTail => {
                // S1: Left Aileron, S2: Elevator, S3: Rudder, S4: Right Aileron (-roll)
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(clamp(pitch, -1.0, 1.0));
                outputs.servo3_us = self.servos[2].apply(clamp(yaw, -1.0, 1.0));
                outputs.servo4_us = self.servos[3].apply(clamp(-roll, -1.0, 1.0));
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::SingleAileronVTail => {
                // S1: Aileron
                // S2: Right V-Tail = 0.5 * Pitch + 0.5 * Yaw
                // S3: Left V-Tail  = 0.5 * Pitch - 0.5 * Yaw
                let s2_norm = clamp(0.5 * pitch + 0.5 * yaw, -1.0, 1.0);
                let s3_norm = clamp(0.5 * pitch - 0.5 * yaw, -1.0, 1.0);
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo3_us = self.servos[2].apply(s3_norm);
                outputs.servo4_us = self.servos[3].neutral();
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::DualAileronVTail => {
                // S1: Left Aileron, S4: Right Aileron (-roll)
                // S2: Right V-Tail = 0.5 * Pitch + 0.5 * Yaw
                // S3: Left V-Tail  = 0.5 * Pitch - 0.5 * Yaw
                let s2_norm = clamp(0.5 * pitch + 0.5 * yaw, -1.0, 1.0);
                let s3_norm = clamp(0.5 * pitch - 0.5 * yaw, -1.0, 1.0);
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo3_us = self.servos[2].apply(s3_norm);
                outputs.servo4_us = self.servos[3].apply(clamp(-roll, -1.0, 1.0));
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
        }

        outputs
    }
}
