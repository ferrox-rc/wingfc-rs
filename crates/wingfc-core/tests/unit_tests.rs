use wingfc_core::math::{Vector3, Quaternion, RAD_TO_DEG};
use wingfc_core::ahrs::MadgwickAHRS;
use wingfc_core::pid::{PidController, PidConfig};
use wingfc_core::mixer::{AirframeMixer, AirframeType};
use wingfc_core::dshot::DshotEncoder;
use wingfc_core::protocol::crsf::{CrsfParser, calculate_crc8_dvb_s2, CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS};

#[test]
fn test_vector_and_quaternion_math() {
    let v1 = Vector3::new(1.0, 2.0, 3.0);
    let v2 = Vector3::new(4.0, 5.0, 6.0);
    assert_eq!(v1.dot(v2), 32.0);

    let q = Quaternion::IDENTITY;
    let (pitch, roll, yaw) = q.to_euler_angles();
    assert!(pitch.abs() < 1e-5);
    assert!(roll.abs() < 1e-5);
    assert!(yaw.abs() < 1e-5);
}

#[test]
fn test_madgwick_ahrs_convergence() {
    let mut ahrs = MadgwickAHRS::new(0.5);
    let dt = 0.005; // 200 Hz
    let gyro = Vector3::ZERO;
    // Stationary flat on table: Accel = 1G pointing straight up Z axis (0, 0, 9.81)
    let accel = Vector3::new(0.0, 0.0, 9.81);

    for _ in 0..200 {
        ahrs.update_6dof(gyro, accel, dt);
    }

    let (pitch, roll, _yaw) = ahrs.get_euler_angles();
    assert!(pitch.abs() * RAD_TO_DEG < 1.0, "Pitch should converge to 0 deg, got {}", pitch * RAD_TO_DEG);
    assert!(roll.abs() * RAD_TO_DEG < 1.0, "Roll should converge to 0 deg, got {}", roll * RAD_TO_DEG);
}

#[test]
fn test_pid_anti_windup() {
    let config = PidConfig {
        kp: 0.2,
        ki: 0.1,
        kd: 0.01,
        max_integral: 0.5,
        dterm_lpf_hz: 30.0,
    };
    let mut pid = PidController::new(config, 0.005);

    // Provide large continuous error (setpoint = 10.0, measurement = 0.0)
    for _ in 0..1000 {
        let _ = pid.update(10.0, 0.0, 0.005);
    }

    // Integral must be clamped to max_integral (0.5)
    assert!((pid.integral() - 0.5).abs() < 1e-4, "PID integral should be clamped at 0.5, got {}", pid.integral());

    // When error goes to 0 (setpoint = 0.0, measurement = 0.0), output should reflect integral
    let output = pid.update(0.0, 0.0, 0.005);
    assert!((output - 0.5).abs() < 1e-4, "Output with 0 error should equal clamped integral 0.5, got {}", output);
}

#[test]
fn test_airframe_mixer_elevon() {
    let mixer = AirframeMixer::new(AirframeType::Elevon);
    
    // Neutral sticks
    let out = mixer.mix(0.0, 0.0, 0.0, 1000);
    assert_eq!(out.servo1_us, 1500);
    assert_eq!(out.servo2_us, 1500);
    assert_eq!(out.motor1_throttle, 1000);

    // Full Pitch up (+1.0)
    let pitch_up = mixer.mix(1.0, 0.0, 0.0, 1000);
    assert_eq!(pitch_up.servo1_us, 1750);
    assert_eq!(pitch_up.servo2_us, 1250);

    // Full Roll right (+1.0)
    let roll_right = mixer.mix(0.0, 1.0, 0.0, 1000);
    assert_eq!(roll_right.servo1_us, 1750);
    assert_eq!(roll_right.servo2_us, 1750);
}

#[test]
fn test_crsf_protocol_parsing() {
    let mut parser = CrsfParser::new();

    // Construct a synthetic 26-byte CRSF packet
    let mut packet = [0u8; 26];
    packet[0] = CRSF_SYNC_BYTE;
    packet[1] = 24; // Length (type + 22 bytes payload + crc)
    packet[2] = CRSF_FRAMETYPE_RC_CHANNELS;
    
    for i in 3..25 {
        packet[i] = 0b11100000;
    }
    let crc = calculate_crc8_dvb_s2(&packet[2..25]);
    packet[25] = crc;

    let mut frame = None;
    for &b in &packet {
        if let Some(f) = parser.parse_byte(b) {
            frame = Some(f);
        }
    }

    assert!(frame.is_some(), "CRSF parser should successfully decode valid packet");
}

#[test]
fn test_dshot_checksum() {
    // 1000 throttle, no telemetry
    let packet = DshotEncoder::encode_packet(1000, false);
    let throttle_back = packet >> 5;
    assert_eq!(throttle_back, 1000);
}
