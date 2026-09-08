use crate::math::{clamp, map_range};

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
    #[inline(always)]
    pub fn apply(&self, normalized: f32) -> u16 {
        // map [-1.0, 1.0] to [min_us, max_us]
        let mut pulse = map_range(
            normalized,
            -1.0,
            1.0,
            self.min_us as f32,
            self.max_us as f32,
        );
        if self.reversed {
            pulse = (self.min_us + self.max_us) as f32 - pulse;
        }
        pulse += self.subtrim_us as f32;
        clamp(pulse, self.min_us as f32, self.max_us as f32) as u16
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AirframeOutputs {
    pub servo1_us: u16,       // Left Elevon / Aileron
    pub servo2_us: u16,       // Right Elevon / Elevator
    pub servo4_us: u16,       // Flap / Aux Servo
    pub servo5_us: u16,       // Flap / Aux Servo
    pub servo6_us: u16,       // Rudder / Aux Servo
    pub motor1_throttle: u16, // 1000..2000 us (or DShot 0..2047)
    pub motor2_throttle: u16, // Twin motor if configured
}

pub struct AirframeMixer {
    pub airframe_type: AirframeType,
    pub servos: [ServoConfig; 5], // index 0..4 corresponds to Servo 1, 2, 4, 5, 6
}

impl AirframeMixer {
    pub fn new(airframe_type: AirframeType) -> Self {
        Self {
            airframe_type,
            servos: [ServoConfig::default(); 5],
        }
    }

    /// Mixes normalized pitch, roll, yaw commands in [-1.0, 1.0] and throttle in [1000, 2000]
    pub fn mix(&self, pitch: f32, roll: f32, yaw: f32, throttle_us: u16) -> AirframeOutputs {
        let mut outputs = AirframeOutputs::default();

        match self.airframe_type {
            AirframeType::Elevon => {
                // Elevon mixing:
                // Left Elevon  =  0.5 * Pitch + 0.5 * Roll
                // Right Elevon = -0.5 * Pitch + 0.5 * Roll
                let s1_norm = clamp(0.5 * pitch + 0.5 * roll, -1.0, 1.0);
                let s2_norm = clamp(-0.5 * pitch + 0.5 * roll, -1.0, 1.0);

                outputs.servo1_us = self.servos[0].apply(s1_norm);
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo4_us = self.servos[2].apply(clamp(yaw, -1.0, 1.0));
                outputs.servo5_us = self.servos[3].apply(0.0);
                outputs.servo6_us = self.servos[4].apply(0.0);

                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::ElevonWithDiffThrust => {
                let s1_norm = clamp(0.5 * pitch + 0.5 * roll, -1.0, 1.0);
                let s2_norm = clamp(-0.5 * pitch + 0.5 * roll, -1.0, 1.0);

                outputs.servo1_us = self.servos[0].apply(s1_norm);
                outputs.servo2_us = self.servos[1].apply(s2_norm);

                let diff = (yaw * 150.0) as i16;
                outputs.motor1_throttle =
                    clamp((throttle_us as i16 - diff) as f32, 1000.0, 2000.0) as u16;
                outputs.motor2_throttle =
                    clamp((throttle_us as i16 + diff) as f32, 1000.0, 2000.0) as u16;
            }
            AirframeType::SingleAileronTTail => {
                // S1: Aileron, S2: Elevator, S4: Rudder
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(clamp(pitch, -1.0, 1.0));
                outputs.servo4_us = self.servos[2].apply(clamp(yaw, -1.0, 1.0));
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::DualAileronTTail => {
                // S1: Left Aileron, S2: Elevator, S4: Rudder, S5: Right Aileron (-roll)
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(clamp(pitch, -1.0, 1.0));
                outputs.servo4_us = self.servos[2].apply(clamp(yaw, -1.0, 1.0));
                outputs.servo5_us = self.servos[3].apply(clamp(-roll, -1.0, 1.0));
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::SingleAileronVTail => {
                // S1: Aileron
                // S2: Right V-Tail = 0.5 * Pitch + 0.5 * Yaw
                // S4: Left V-Tail  = 0.5 * Pitch - 0.5 * Yaw
                let s2_norm = clamp(0.5 * pitch + 0.5 * yaw, -1.0, 1.0);
                let s4_norm = clamp(0.5 * pitch - 0.5 * yaw, -1.0, 1.0);
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo4_us = self.servos[2].apply(s4_norm);
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
            AirframeType::DualAileronVTail => {
                // S1: Left Aileron, S5: Right Aileron (-roll)
                // S2: Right V-Tail = 0.5 * Pitch + 0.5 * Yaw
                // S4: Left V-Tail  = 0.5 * Pitch - 0.5 * Yaw
                let s2_norm = clamp(0.5 * pitch + 0.5 * yaw, -1.0, 1.0);
                let s4_norm = clamp(0.5 * pitch - 0.5 * yaw, -1.0, 1.0);
                outputs.servo1_us = self.servos[0].apply(clamp(roll, -1.0, 1.0));
                outputs.servo2_us = self.servos[1].apply(s2_norm);
                outputs.servo4_us = self.servos[2].apply(s4_norm);
                outputs.servo5_us = self.servos[3].apply(clamp(-roll, -1.0, 1.0));
                outputs.motor1_throttle = throttle_us;
                outputs.motor2_throttle = throttle_us;
            }
        }

        outputs
    }
}
