use wingfc_core::ahrs::MadgwickAHRS;
use wingfc_core::dshot::DshotEncoder;
use wingfc_core::math::{Quaternion, Vector3, RAD_TO_DEG};
use wingfc_core::mixer::{AirframeMixer, AirframeType};
use wingfc_core::pid::{PidConfig, PidController};
use wingfc_core::protocol::crsf::{
    calculate_crc8_dvb_s2, CrsfParser, CRSF_FRAMETYPE_RC_CHANNELS, CRSF_SYNC_BYTE,
};
use wingfc_core::protocol::ibus::{IbusParser, IBUS_HEADER1, IBUS_HEADER2, IBUS_PACKET_SIZE};
use wingfc_core::protocol::RcChannels;

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
    let accel = Vector3::new(0.0, 0.0, 9.81);

    for _ in 0..200 {
        ahrs.update_6dof(gyro, accel, dt);
    }

    let (pitch, roll, _yaw) = ahrs.get_euler_angles();
    assert!(
        pitch.abs() * RAD_TO_DEG < 1.0,
        "Pitch should converge to 0 deg, got {}",
        pitch * RAD_TO_DEG
    );
    assert!(
        roll.abs() * RAD_TO_DEG < 1.0,
        "Roll should converge to 0 deg, got {}",
        roll * RAD_TO_DEG
    );
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

    for _ in 0..1000 {
        let _ = pid.update(10.0, 0.0, 0.005);
    }

    assert!(
        (pid.integral() - 0.5).abs() < 1e-4,
        "PID integral should be clamped at 0.5, got {}",
        pid.integral()
    );

    let output = pid.update(0.0, 0.0, 0.005);
    assert!(
        (output - 0.5).abs() < 1e-4,
        "Output with 0 error should equal clamped integral 0.5, got {}",
        output
    );
}

#[test]
fn test_airframe_mixer_elevon() {
    let mixer = AirframeMixer::new(AirframeType::Elevon);

    let out = mixer.mix(0.0, 0.0, 0.0, 1000);
    assert_eq!(out.servo1_us, 1500);
    assert_eq!(out.servo2_us, 1500);
    assert_eq!(out.motor1_throttle, 1000);

    let pitch_up = mixer.mix(1.0, 0.0, 0.0, 1000);
    assert_eq!(pitch_up.servo1_us, 1750);
    assert_eq!(pitch_up.servo2_us, 1250);

    let roll_right = mixer.mix(0.0, 1.0, 0.0, 1000);
    assert_eq!(roll_right.servo1_us, 1750);
    assert_eq!(roll_right.servo2_us, 1750);
}

#[test]
fn test_crsf_protocol_parsing() {
    let mut parser = CrsfParser::new();

    let mut packet = [0u8; 26];
    packet[0] = CRSF_SYNC_BYTE;
    packet[1] = 24;
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

    assert!(
        frame.is_some(),
        "CRSF parser should successfully decode valid packet"
    );
    let rc: RcChannels = frame.unwrap().into();
    assert_eq!(rc.channels[0], 1020);
}

#[test]
fn test_ibus_protocol_parsing() {
    let mut parser = IbusParser::new();

    // Construct valid 32-byte iBus frame
    let mut packet = [0u8; IBUS_PACKET_SIZE];
    packet[0] = IBUS_HEADER1; // 0x20
    packet[1] = IBUS_HEADER2; // 0x40

    // Set 14 channels to 1500 us (0x05DC in little endian: [0xDC, 0x05])
    for ch in 0..14 {
        packet[2 + ch * 2] = 0xDC;
        packet[2 + ch * 2 + 1] = 0x05;
    }

    // Compute checksum: 0xFFFF - sum(bytes[0..30])
    let mut sum: u32 = 0;
    for &b in &packet[0..30] {
        sum += b as u32;
    }
    let checksum = (0xFFFF - (sum & 0xFFFF)) as u16;
    packet[30] = (checksum & 0xFF) as u8;
    packet[31] = (checksum >> 8) as u8;

    let mut frame = None;
    for &b in &packet {
        if let Some(f) = parser.parse_byte(b) {
            frame = Some(f);
        }
    }

    assert!(
        frame.is_some(),
        "iBus parser should successfully decode valid packet"
    );
    let f = frame.unwrap();
    assert_eq!(f.channels[0], 1500);
    assert_eq!(f.channels[13], 1500);

    let rc: RcChannels = f.into();
    assert_eq!(rc.channels[0], 1500);
}

#[test]
fn test_dshot_checksum() {
    let packet = DshotEncoder::encode_packet(1000, false);
    let throttle_back = packet >> 5;
    assert_eq!(throttle_back, 1000);
}

#[test]
fn test_arming_safety_logic() {
    use wingfc_core::safety::{ArmingSafety, ArmingStatus};

    let mut safety = ArmingSafety::new();

    // 1. Boot with arm switch already HIGH -> must be REFUSED
    let status = safety.update(1900, 1000, 1000, false);
    assert_eq!(status, ArmingStatus::RefusedSwitchArmedAtBoot);
    assert!(!safety.is_armed());

    // 2. Toggle switch to DISARM -> should now be Disarmed and unlocked
    let status = safety.update(1000, 1000, 1000, false);
    assert_eq!(status, ArmingStatus::Disarmed);
    assert!(!safety.is_armed());

    // 3. Try to arm while throttle is raised (e.g. 1300us) -> must be REFUSED
    let status = safety.update(1900, 1000, 1300, false);
    assert_eq!(status, ArmingStatus::RefusedThrottleRaised);
    assert!(!safety.is_armed());

    // 4. Arm with throttle at zero (1000us) and mode < 1500 (Angle mode) -> ARMED ANGLE
    let status = safety.update(1900, 1000, 1000, false);
    assert_eq!(status, ArmingStatus::ArmedAngle);
    assert!(safety.is_armed());

    // 5. While armed, throttle raised to full (2000us) -> stays armed
    let status = safety.update(1900, 1000, 2000, false);
    assert_eq!(status, ArmingStatus::ArmedAngle);
    assert!(safety.is_armed());

    // 6. Switch mode to Acro (mode >= 1500) -> ARMED ACRO
    let status = safety.update(1900, 1800, 1500, false);
    assert_eq!(status, ArmingStatus::ArmedAcro);
    assert!(safety.is_armed());

    // 7. Signal lost -> enters FAILSAFE immediately and disarms
    let status = safety.update(1900, 1800, 1500, true);
    assert_eq!(status, ArmingStatus::Failsafe);
    assert!(!safety.is_armed());

    // 8. Signal restored but switch still HIGH and throttle raised -> rejected
    let status = safety.update(1900, 1800, 1500, false);
    assert_eq!(status, ArmingStatus::RefusedThrottleRaised);
    assert!(!safety.is_armed());
}
