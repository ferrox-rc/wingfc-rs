use wingfc_core::ahrs::VqfAHRS;
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

    // Cross product & operations
    let cross = v1.cross(v2);
    assert_eq!(cross.x, -3.0);
    assert_eq!(cross.y, 6.0);
    assert_eq!(cross.z, -3.0);
    assert_eq!(v1 + v2, Vector3::new(5.0, 7.0, 9.0));
    assert_eq!(v2 - v1, Vector3::new(3.0, 3.0, 3.0));
    assert_eq!(v1 * 2.0, Vector3::new(2.0, 4.0, 6.0));
    let norm = Vector3::new(3.0, 4.0, 0.0).norm();
    assert!((norm - 5.0).abs() < 1e-5);
    let normalized = Vector3::new(0.0, 10.0, 0.0).normalize();
    assert!((normalized.y - 1.0).abs() < 1e-5);
    assert_eq!(Vector3::ZERO.normalize(), Vector3::ZERO);

    // Quaternion operations
    let q = Quaternion::IDENTITY;
    assert_eq!(q.conjugate(), Quaternion::IDENTITY);
    assert!((q.norm() - 1.0).abs() < 1e-5);
    assert_eq!(q.multiply(Quaternion::IDENTITY), Quaternion::IDENTITY);
    let q2 = Quaternion::new(2.0, 0.0, 0.0, 0.0).normalize();
    assert!((q2.w - 1.0).abs() < 1e-5);
    assert_eq!(
        Quaternion::new(0.0, 0.0, 0.0, 0.0).normalize(),
        Quaternion::IDENTITY
    );

    let (pitch, roll, yaw) = q.to_euler_angles();
    assert!(pitch.abs() < 1e-5);
    assert!(roll.abs() < 1e-5);
    assert!(yaw.abs() < 1e-5);
}

#[test]
fn test_vqf_ahrs_convergence() {
    let mut ahrs = VqfAHRS::new(3.5, 9.0, 0.005);
    let dt = 0.005; // 200 Hz
    let gyro = Vector3::ZERO;
    let accel = Vector3::new(0.0, 0.0, 9.81);

    for _ in 0..200 {
        ahrs.update_6dof(gyro, accel, dt);
    }

    let (pitch, roll, _yaw) = ahrs.euler_angles();
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
        kff: 0.0,
        max_integral: 0.5,
        dterm_lpf_hz: 30.0,
        ff_lpf_hz: 45.0,
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

    for byte in packet[3..25].iter_mut() {
        *byte = 0b11100000;
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

#[test]
fn test_arming_safety_ground_signal_lost() {
    use wingfc_core::safety::{ArmingSafety, ArmingStatus};

    let mut safety = ArmingSafety::new();

    // 1. Boot on bench with signal lost -> must be Disarmed (NOT Failsafe)
    let status = safety.update(1000, 1000, 1000, true);
    assert_eq!(status, ArmingStatus::Disarmed);
    assert!(!safety.is_armed());

    // 2. Switch flipped to ARM while signal lost -> must stay Disarmed
    let status = safety.update(1900, 1000, 1000, true);
    assert_eq!(status, ArmingStatus::Disarmed);
    assert!(!safety.is_armed());

    // 3. Signal acquired with switch disarmed -> Disarmed and ready
    let status = safety.update(1000, 1000, 1000, false);
    assert_eq!(status, ArmingStatus::Disarmed);
    assert!(!safety.is_armed());

    // 4. Arm safely with zero throttle -> ArmedAngle
    let status = safety.update(1900, 1000, 1000, false);
    assert_eq!(status, ArmingStatus::ArmedAngle);
    assert!(safety.is_armed());

    // 5. In-flight signal lost -> enters Failsafe immediately
    let status = safety.update(1900, 1000, 1000, true);
    assert_eq!(status, ArmingStatus::Failsafe);
    assert!(!safety.is_armed());

    // 6. Subsequent cycles with signal still lost -> latches Failsafe
    let status = safety.update(1900, 1000, 1000, true);
    assert_eq!(status, ArmingStatus::Failsafe);
    assert!(!safety.is_armed());
}

#[test]
fn test_feedforward_response() {
    let cfg = PidConfig {
        kff: 0.10,
        kp: 0.0,
        ki: 0.0,
        kd: 0.0,
        ..Default::default()
    };
    let mut pid = PidController::new(cfg, 0.002);

    // Rapid stick movement: setpoint steps from 0.0 to 10.0
    let out_step = pid.update(10.0, 0.0, 0.002);
    assert!(
        out_step > 0.0,
        "Feedforward should immediately produce positive output on stick step"
    );

    // Hold stick steady at 10.0 for 50 cycles: Feedforward must decay to zero
    let mut out_steady = 0.0;
    for _ in 0..50 {
        out_steady = pid.update(10.0, 0.0, 0.002);
    }
    assert!(
        out_steady.abs() < 0.01,
        "Feedforward must return to zero while stick is held steady"
    );
}

#[test]
fn test_hand_autolaunch_flow() {
    use wingfc_core::autolaunch::{AutolaunchController, AutolaunchState};

    let mut launcher = AutolaunchController::default();
    launcher.prime();
    assert_eq!(launcher.state(), AutolaunchState::WaitingForThrow);

    // 1. In pilot hand: normal handling (accel < threshold)
    let (_, _, throttle) = launcher.update(5.0, 0.0, 0.0, 0.01);
    assert_eq!(throttle, 1000);
    assert_eq!(launcher.state(), AutolaunchState::WaitingForThrow);

    // 2. Forward throw detected (>13.5 m/s^2 for >0.05s)
    for _ in 0..6 {
        let _ = launcher.update(15.0, 0.0, 0.0, 0.01);
    }
    assert_eq!(launcher.state(), AutolaunchState::MotorDelay);

    // 3. Finger-safe motor delay (200ms) - throttle MUST remain 1000us
    for _ in 0..15 {
        let (_, _, throttle) = launcher.update(0.0, 0.0, 0.0, 0.01);
        assert_eq!(throttle, 1000);
    }

    // 4. Delay passes -> enters Climbing mode and ramps throttle
    for _ in 0..10 {
        let (_, _, _) = launcher.update(0.0, 0.0, 0.0, 0.01);
    }
    assert_eq!(launcher.state(), AutolaunchState::Climbing);

    // 5. Pilot moves stick to abort autolaunch -> returns to Completed
    let (_, _, _) = launcher.update(0.0, 0.50, 0.0, 0.01);
    assert_eq!(launcher.state(), AutolaunchState::Completed);
}

#[cfg(feature = "compass")]
#[test]
fn test_9dof_vqf_heading() {
    let mut ahrs = VqfAHRS::new(3.5, 9.0, 0.005);
    let dt = 0.005;
    let dt_mag = 0.05; // 20 Hz
    let gyro = Vector3::ZERO;
    let accel = Vector3::new(0.0, 0.0, 9.81);
    // Magnetic North vector pointing purely along X axis (+X = North)
    let mag = Vector3::new(30.0, 0.0, 0.0);

    for step in 0..200 {
        ahrs.update_6dof(gyro, accel, dt);
        if step % 10 == 0 {
            ahrs.update_mag(mag, dt_mag);
        }
    }

    let (pitch, roll, yaw) = ahrs.euler_angles();
    assert!(pitch.abs() * RAD_TO_DEG < 1.0);
    assert!(roll.abs() * RAD_TO_DEG < 1.0);
    assert!(
        yaw.abs() * RAD_TO_DEG < 2.0,
        "Heading should converge to North (0 deg), got {}",
        yaw * RAD_TO_DEG
    );
}

#[cfg(feature = "compass")]
#[test]
fn test_vqf_magnetic_disturbance_rejection() {
    let mut ahrs = VqfAHRS::new(3.5, 9.0, 0.005);
    let dt = 0.005;
    let dt_mag = 0.05;
    let gyro = Vector3::ZERO;
    let accel = Vector3::new(0.0, 0.0, 9.81);
    let mag_clean = Vector3::new(30.0, 0.0, -10.0);

    // 1. Establish clean reference
    for step in 0..100 {
        ahrs.update_6dof(gyro, accel, dt);
        if step % 10 == 0 {
            ahrs.update_mag(mag_clean, dt_mag);
        }
    }
    assert!(!ahrs.mag_dist_detected);

    let (p_before, r_before, y_before) = ahrs.euler_angles();

    // 2. Inject massive motor current / magnetic spike (3x norm + distorted dip)
    let mag_spiked = Vector3::new(90.0, 50.0, 80.0);
    for step in 0..100 {
        ahrs.update_6dof(gyro, accel, dt);
        if step % 10 == 0 {
            ahrs.update_mag(mag_spiked, dt_mag);
        }
    }

    // Magnetic disturbance MUST be detected
    assert!(ahrs.mag_dist_detected, "Disturbance must be flagged");

    // Pitch and roll MUST NOT be corrupted by the magnetic spike
    let (p_after, r_after, y_after) = ahrs.euler_angles();
    assert!(
        (p_after - p_before).abs() * RAD_TO_DEG < 0.1,
        "Pitch should be immune to magnetic spike"
    );
    assert!(
        (r_after - r_before).abs() * RAD_TO_DEG < 0.1,
        "Roll should be immune to magnetic spike"
    );
    assert!(
        (y_after - y_before).abs() * RAD_TO_DEG < 0.5,
        "Heading should be frozen/rejected during spike"
    );
}

#[test]
fn test_vqf_rest_detection_and_bias_cancellation() {
    let mut ahrs = VqfAHRS::new(3.5, 9.0, 0.005);
    let dt = 0.005;
    let accel = Vector3::new(0.0, 0.0, 9.81);

    // Constant uncalibrated gyro turn-on bias (e.g. 1.0 deg/s around z)
    let bias_rate = 1.0 * (core::f32::consts::PI / 180.0);
    let raw_gyro = Vector3::new(0.0, 0.0, bias_rate);

    // Run stationary for 4 seconds (> 1.5s rest_min_t)
    for _ in 0..800 {
        ahrs.update_6dof(raw_gyro, accel, dt);
    }

    assert!(
        ahrs.rest_detected,
        "Rest should be detected when stationary"
    );
    assert!(
        (ahrs.bias.z - bias_rate).abs() < 0.1 * (core::f32::consts::PI / 180.0),
        "Z-bias estimate should converge near actual bias, got {} deg/s",
        ahrs.bias.z * RAD_TO_DEG
    );
}

#[test]
fn test_vqf_centripetal_turn_compensation() {
    let mut ahrs = VqfAHRS::new(3.5, 9.0, 0.005);
    let dt = 0.005;
    let roll_deg = 30.0f32;
    let roll_rad = roll_deg * core::f32::consts::PI / 180.0;
    let v_mps = 18.0f32;
    let g = 9.80665f32;

    // Coordinated turn: psi_dot = (g * tan(phi)) / v
    let tan_phi = libm::tanf(roll_rad);
    let psi_dot = (g * tan_phi) / v_mps;
    // Body rates for banked turn (pulling elevator to track turn plane):
    let gyro = Vector3::new(
        0.0,
        psi_dot * libm::sinf(roll_rad),
        psi_dot * libm::cosf(roll_rad),
    );

    // In a coordinated turn (zero slip), specific force is along aircraft Z: f_z = g / cos(phi)
    let accel_measured = Vector3::new(0.0, 0.0, g / libm::cosf(roll_rad));

    // Initialize roll attitude to 30 deg
    let half_phi = 0.5 * roll_rad;
    ahrs.gyr_quat = Quaternion::new(libm::cosf(half_phi), libm::sinf(half_phi), 0.0, 0.0);

    // Run for 5 seconds (1000 steps) with GPS centripetal aiding
    for _ in 0..1000 {
        ahrs.update_6dof_centripetal_aided(gyro, accel_measured, v_mps, dt);
    }

    let (_, roll, _) = ahrs.euler_angles();
    let roll_err_deg = (roll * RAD_TO_DEG - roll_deg).abs();
    assert!(
        roll_err_deg < 1.0,
        "With GPS aiding, roll should stay at 30 deg during sustained turn, got {} deg (err {} deg)",
        roll * RAD_TO_DEG,
        roll_err_deg
    );
}

#[cfg(feature = "baro")]
#[test]
fn test_vertical_kalman_filter() {
    use wingfc_core::vertical_kf::VerticalKalmanFilter;

    let mut vkf = VerticalKalmanFilter::default();
    vkf.reset(0.0);

    // Simulate ascending at +2.0 m/s for 1 second (500Hz control loop)
    for step in 0..500 {
        vkf.predict(0.0, 0.002);

        // Inject barometer measurement at 50Hz (every 10 steps)
        if step % 10 == 0 {
            let true_alt = (step as f32) * 0.002 * 2.0;
            vkf.update_baro(true_alt);
        }
    }

    assert!(
        (vkf.altitude_m - 2.0).abs() < 0.15,
        "Estimated altitude should be ~2.0m, got {}",
        vkf.altitude_m
    );
    assert!(
        (vkf.velocity_ms - 2.0).abs() < 0.25,
        "Estimated climb rate should be ~2.0 m/s, got {}",
        vkf.velocity_ms
    );
}

#[cfg(feature = "baro")]
#[test]
fn test_altitude_safety_interventions() {
    use wingfc_core::altitude_safety::{AltitudeIntervention, AltitudeSafetyController};

    let controller = AltitudeSafetyController::default();

    // 1. Normal cruising flight at 50m AGL
    let (action, pitch, throttle) = controller.evaluate(50.0, 0.0, 0.0, 0.0, 1500);
    assert_eq!(action, AltitudeIntervention::None);
    assert_eq!(pitch, 0.0);
    assert_eq!(throttle, 1500);

    // 2. Hard Deck breach: Plane drops below 15m (10m AGL) with negative sink rate
    let (action, pitch, throttle) = controller.evaluate(10.0, -2.0, -0.10, -0.10, 1300);
    assert_eq!(action, AltitudeIntervention::HardDeckPullUp);
    assert!(
        pitch > 0.30,
        "Hard Deck must command positive pitch pull-up"
    );
    assert!(
        throttle >= 1650,
        "Hard Deck must command safe climb throttle"
    );

    // 3. Legal Ceiling: Plane climbs above 120m (125m AGL)
    let (action, pitch, throttle) = controller.evaluate(125.0, 2.0, 0.20, 0.20, 1800);
    assert_eq!(action, AltitudeIntervention::CeilingPitchDown);
    assert!(pitch <= -0.20, "Ceiling limiter must pitch down");
    assert!(throttle <= 1450, "Ceiling limiter must reduce throttle");

    // 4. Stall detect: Plane is level (0.0 pitch) but sinking rapidly (-4.5 m/s)
    let (action, pitch, throttle) = controller.evaluate(60.0, -4.5, 0.0, 0.10, 1500);
    assert_eq!(action, AltitudeIntervention::StallRecovery);
    assert!(
        pitch < 0.0,
        "Stall recovery must drop nose to regain airspeed"
    );
    assert_eq!(throttle, 1900, "Stall recovery must command full throttle");
}

#[cfg(feature = "gps")]
#[test]
fn test_nmea_gga_rmc_parsing() {
    use wingfc_core::gps::{GpsFixType, NmeaParser};

    let mut parser = NmeaParser::new();

    // Standard GGA sentence: 48 deg 07.038 min N, 11 deg 31.000 min E, Fix=1, 8 sats, HDOP 0.9, Alt 545.4m
    let gga_payload = b"GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    let mut gga_cs: u8 = 0;
    for &b in gga_payload {
        gga_cs ^= b;
    }
    let gga_str = format!(
        "${}*{:02X}
",
        std::str::from_utf8(gga_payload).unwrap(),
        gga_cs
    );
    let mut updated = false;
    for &b in gga_str.as_bytes() {
        if parser.feed_byte(b).is_some() {
            updated = true;
        }
    }
    assert!(
        updated,
        "NMEA parser should return updated GPS fix for valid GGA"
    );

    let data = parser.data();
    // 48 + 07.038/60 = 48.1173 deg
    assert!((data.latitude_deg - 48.1173).abs() < 0.001);
    // 11 + 31.0/60 = 11.51667 deg
    assert!((data.longitude_deg - 11.5166).abs() < 0.001);
    assert_eq!(data.satellites, 8);
    assert_eq!(data.fix_type, GpsFixType::Fix3D);
    assert!((data.altitude_msl_m - 545.4).abs() < 0.1);
    assert!((data.hdop - 0.9).abs() < 0.05);

    // Standard RMC sentence: Status A, 22.4 knots ground speed, 84.4 deg course
    let rmc_payload = b"GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
    let mut cs: u8 = 0;
    for &b in rmc_payload {
        cs ^= b;
    }
    let rmc_str = format!(
        "${}*{:02X}
",
        std::str::from_utf8(rmc_payload).unwrap(),
        cs
    );
    updated = false;
    for &b in rmc_str.as_bytes() {
        if parser.feed_byte(b).is_some() {
            updated = true;
        }
    }
    assert!(
        updated,
        "NMEA parser should return updated GPS fix for valid RMC"
    );

    let data2 = parser.data();
    // 22.4 knots * 0.514444 = 11.52 m/s
    assert!((data2.ground_speed_m_s - 11.52).abs() < 0.1);
    assert!((data2.course_deg - 84.4).abs() < 0.1);
}

#[cfg(feature = "gps")]
#[test]
fn test_gps_navigation_home_latching_and_rth() {
    use wingfc_core::gps::{GpsData, GpsFixType, GpsNavigation};

    let mut nav = GpsNavigation::new(6, 2.5, 30.0);
    assert!(!nav.is_home_latched());

    // 1. Initial 2D fix with low satellite count -> must NOT latch home
    let mut gps = GpsData {
        latitude_deg: 37.7749,
        longitude_deg: -122.4194,
        altitude_msl_m: 10.0,
        ground_speed_m_s: 0.0,
        course_deg: 0.0,
        satellites: 4,
        hdop: 3.5,
        fix_type: GpsFixType::Fix2D,
    };
    assert!(!nav.try_latch_home(&gps));
    assert!(!nav.is_home_latched());

    // 2. High quality 3D fix (8 satellites, HDOP 1.1) -> latches home
    gps.satellites = 8;
    gps.hdop = 1.1;
    gps.fix_type = GpsFixType::Fix3D;
    assert!(nav.try_latch_home(&gps));
    assert!(nav.is_home_latched());

    // 3. Aircraft flies North-East (e.g. +1.0km North, +1.0km East)
    let current_pos = GpsData {
        latitude_deg: 37.7839,
        longitude_deg: -122.408,
        altitude_msl_m: 50.0,
        ground_speed_m_s: 15.0,
        course_deg: 45.0,
        satellites: 10,
        hdop: 0.9,
        fix_type: GpsFixType::Fix3D,
    };

    let guidance = nav
        .calculate_rth_guidance(&current_pos)
        .expect("RTH guidance must be available when home is latched");

    // Bearing to home should be South-West (~225 degrees)
    assert!(
        guidance.bearing_to_home_deg > 210.0 && guidance.bearing_to_home_deg < 240.0,
        "Bearing to home should be ~225 deg, got {}",
        guidance.bearing_to_home_deg
    );

    // Distance should be approximately 1.4 km (1414 m)
    assert!(
        guidance.distance_to_home_m > 1200.0 && guidance.distance_to_home_m < 1600.0,
        "Distance to home should be ~1400m, got {}",
        guidance.distance_to_home_m
    );

    // Heading error relative to current track (flying at 45 deg, needs to turn to ~225 deg): error should be ~ -180 / +180 deg
    assert!(guidance.heading_error_deg.abs() > 160.0);

    // Target altitude should be home altitude (10.0m) + safe RTH climb margin (30.0m) = 40.0m
    assert!((guidance.target_altitude_m - 40.0).abs() < 0.1);
}

#[test]
fn test_configurable_arming_thresholds() {
    use wingfc_core::safety::{ArmingSafety, ArmingStatus};

    // Configured with custom thresholds: Arm switch > 1600us, Throttle <= 1100us
    let mut safety = ArmingSafety::with_thresholds(1600, 1100);
    assert_eq!(safety.arm_switch_threshold(), 1600);
    assert_eq!(safety.throttle_zero_threshold(), 1100);

    // Initial disarm pulse to satisfy boot-switch interlock
    safety.update(1000, 1000, 1000, false);

    // 1. Arm switch at 1550us (below 1600 threshold) -> remains Disarmed
    let status = safety.update(1550, 1000, 1000, false);
    assert_eq!(status, ArmingStatus::Disarmed);
    assert!(!safety.is_armed());

    // 2. Throttle at 1120us (above 1100 threshold) with switch at 1700us -> RefusedThrottleRaised
    let status = safety.update(1700, 1000, 1120, false);
    assert_eq!(status, ArmingStatus::RefusedThrottleRaised);
    assert!(!safety.is_armed());

    // 3. Throttle at 1080us (below 1100 threshold) with switch at 1700us -> successfully arms
    let status = safety.update(1700, 1000, 1080, false);
    assert_eq!(status, ArmingStatus::ArmedAngle);
    assert!(safety.is_armed());
}

#[test]
fn test_crsf_telemetry_encoding() {
    use wingfc_core::protocol::crsf::{
        calculate_crc8_dvb_s2, CrsfTelemetryEncoder, CRSF_FRAMETYPE_ATTITUDE,
        CRSF_FRAMETYPE_BARO_ALTITUDE, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAMETYPE_FLIGHT_MODE,
        CRSF_FRAMETYPE_GPS, CRSF_SYNC_BYTE,
    };

    let mut buf = [0u8; 32];

    // 1. Flight Mode: "ANGLE"
    let len = CrsfTelemetryEncoder::encode_flight_mode("ANGLE", &mut buf);
    assert_eq!(len, 10); // Sync(1) + Len(1) + Type(1) + "ANGLE "(6) + CRC(1) = 10
    assert_eq!(buf[0], CRSF_SYNC_BYTE);
    assert_eq!(buf[1], 8); // Type(1) + 6 + CRC(1) = 8
    assert_eq!(buf[2], CRSF_FRAMETYPE_FLIGHT_MODE);
    assert_eq!(&buf[3..9], b"ANGLE ");
    assert_eq!(buf[9], calculate_crc8_dvb_s2(&buf[2..9]));

    // 2. Vehicle Attitude: pitch=0.1 rad, roll=-0.2 rad, yaw=1.5 rad
    let len = CrsfTelemetryEncoder::encode_attitude(0.1, -0.2, 1.5, &mut buf);
    assert_eq!(len, 10); // Sync(1) + Len(1) + Type(1) + Payload(6) + CRC(1) = 10
    assert_eq!(buf[0], CRSF_SYNC_BYTE);
    assert_eq!(buf[1], 8);
    assert_eq!(buf[2], CRSF_FRAMETYPE_ATTITUDE);
    let pitch = i16::from_be_bytes([buf[3], buf[4]]);
    let roll = i16::from_be_bytes([buf[5], buf[6]]);
    let yaw = i16::from_be_bytes([buf[7], buf[8]]);
    assert_eq!(pitch, 1000);
    assert_eq!(roll, -2000);
    assert_eq!(yaw, 15000);
    assert_eq!(buf[9], calculate_crc8_dvb_s2(&buf[2..9]));

    // 3. Battery Sensor: 12.6V (126 dv), 15.4A (154 da), 850 mAh, 78%
    let len = CrsfTelemetryEncoder::encode_battery(126, 154, 850, 78, &mut buf);
    assert_eq!(len, 12); // Sync(1) + Len(1) + Type(1) + Payload(8) + CRC(1) = 12
    assert_eq!(buf[0], CRSF_SYNC_BYTE);
    assert_eq!(buf[1], 10);
    assert_eq!(buf[2], CRSF_FRAMETYPE_BATTERY_SENSOR);
    let v = u16::from_be_bytes([buf[3], buf[4]]);
    let i = u16::from_be_bytes([buf[5], buf[6]]);
    let mah = ((buf[7] as u32) << 16) | ((buf[8] as u32) << 8) | (buf[9] as u32);
    assert_eq!(v, 126);
    assert_eq!(i, 154);
    assert_eq!(mah, 850);
    assert_eq!(buf[10], 78);
    assert_eq!(buf[11], calculate_crc8_dvb_s2(&buf[2..11]));

    // 4. GPS Navigation: lat=37.7749, lon=-122.4194, speed=12.5 m/s, heading=180.0 deg, alt=120m, sats=14
    let len =
        CrsfTelemetryEncoder::encode_gps(37.7749, -122.4194, 12.5, 180.0, 120.0, 14, &mut buf);
    assert_eq!(len, 19); // Sync(1) + Len(1) + Type(1) + Payload(15) + CRC(1) = 19
    assert_eq!(buf[0], CRSF_SYNC_BYTE);
    assert_eq!(buf[1], 17);
    assert_eq!(buf[2], CRSF_FRAMETYPE_GPS);
    let lat = i32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]);
    let lon = i32::from_be_bytes([buf[7], buf[8], buf[9], buf[10]]);
    let speed = u16::from_be_bytes([buf[11], buf[12]]);
    let hdg = u16::from_be_bytes([buf[13], buf[14]]);
    let alt = u16::from_be_bytes([buf[15], buf[16]]);
    assert_eq!(lat, 377749000);
    assert_eq!(lon, -1224194000);
    assert_eq!(speed, 450); // 12.5 * 36.0 = 450 km/h * 10
    assert_eq!(hdg, 18000); // 180.0 * 100
    assert_eq!(alt, 1120); // 120 + 1000m offset
    assert_eq!(buf[17], 14); // 14 satellites
    assert_eq!(buf[18], calculate_crc8_dvb_s2(&buf[2..18]));

    // 5. Barometer Altitude: alt=250m, vario=150 cm/s
    let len = CrsfTelemetryEncoder::encode_baro_altitude(250.0, 150, &mut buf);
    assert_eq!(len, 8); // Sync(1) + Len(1) + Type(1) + Payload(4) + CRC(1) = 8
    assert_eq!(buf[0], CRSF_SYNC_BYTE);
    assert_eq!(buf[1], 6);
    assert_eq!(buf[2], CRSF_FRAMETYPE_BARO_ALTITUDE);
    let baro_alt = u16::from_be_bytes([buf[3], buf[4]]);
    let vario = i16::from_be_bytes([buf[5], buf[6]]);
    assert_eq!(baro_alt, 10250); // 250 + 10000m offset
    assert_eq!(vario, 150);
    assert_eq!(buf[7], calculate_crc8_dvb_s2(&buf[2..7]));
}

#[test]
fn test_filter_pt1_and_biquad() {
    use wingfc_core::filter::{BiquadFilter, Pt1Filter};

    let mut pt1 = Pt1Filter::new();
    pt1.init(20.0, 0.002); // 20 Hz cutoff, 500 Hz sample rate (dt=2ms)
    assert_eq!(pt1.state(), 0.0);

    for _ in 0..100 {
        pt1.update(1.0);
    }
    assert!(
        (pt1.state() - 1.0).abs() < 0.01,
        "PT1 should settle near 1.0"
    );

    pt1.reset(0.0);
    assert_eq!(pt1.state(), 0.0);

    // Biquad low-pass
    let mut bq = BiquadFilter::new();
    bq.init(50.0, 500.0);
    let mut out = 0.0;
    for _ in 0..100 {
        out = bq.update(1.0);
    }
    assert!(
        (out - 1.0).abs() < 0.05,
        "Biquad should settle near 1.0, got {}",
        out
    );

    bq.reset();
}

#[test]
fn test_dshot_encoder_full_pipeline() {
    use wingfc_core::dshot::{DshotEncoder, DSHOT_MAX_THROTTLE, DSHOT_MIN_THROTTLE};

    // Min throttle clamp
    let pkt = DshotEncoder::encode_packet(10, false);
    let expected = DshotEncoder::encode_packet(DSHOT_MIN_THROTTLE, false);
    assert_eq!(pkt, expected);

    // Max throttle clamp
    let pkt_max = DshotEncoder::encode_packet(3000, false);
    let expected_max = DshotEncoder::encode_packet(DSHOT_MAX_THROTTLE, false);
    assert_eq!(pkt_max, expected_max);

    // to_spi_buffer
    let mut buf = [0u8; 16];
    DshotEncoder::to_spi_buffer(pkt, &mut buf);
    assert_eq!(buf.len(), 16);
}

#[test]
fn test_airframe_mixer_all_modes() {
    use wingfc_core::mixer::{AirframeMixer, AirframeType, ServoConfig};

    // Test servo config mapping and reversing
    let servo = ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 50,
        reversed: true,
    };
    let center = servo.apply(0.0);
    assert_eq!(center, 1550); // 1500 reversed + 50 subtrim

    // Elevon with differential thrust
    let mixer = AirframeMixer::new(AirframeType::ElevonWithDiffThrust);
    let out = mixer.mix(0.0, 0.0, 0.5, 1500); // 1500us throttle, 50% positive yaw (0.5 * 0.15 * 1000 = 75us)
    assert!(
        out.motor2_throttle > out.motor1_throttle,
        "Motor 2 should spin faster on positive yaw"
    );
    assert_eq!(out.motor1_throttle, 1425);
    assert_eq!(out.motor2_throttle, 1575);

    // Test custom differential thrust weighting (e.g. 0.25 -> 250us max authority)
    let mixer_custom = AirframeMixer::with_diff_thrust(AirframeType::ElevonWithDiffThrust, 0.25);
    let out_custom = mixer_custom.mix(0.0, 0.0, 0.5, 1500); // 0.5 * 0.25 * 1000 = 125us
    assert_eq!(out_custom.motor1_throttle, 1375);
    assert_eq!(out_custom.motor2_throttle, 1625);

    // Single Aileron T-Tail
    let mixer_tt = AirframeMixer::new(AirframeType::SingleAileronTTail);
    let out_tt = mixer_tt.mix(0.2, -0.3, 0.1, 1600);
    assert_eq!(out_tt.motor1_throttle, 1600);
    assert!(out_tt.servo1_us != 1500); // Aileron
    assert!(out_tt.servo2_us != 1500); // Elevator
    assert!(out_tt.servo3_us != 1500); // Rudder

    // Dual Aileron T-Tail (differential ailerons)
    let mixer_dtt = AirframeMixer::new(AirframeType::DualAileronTTail);
    let out_dtt = mixer_dtt.mix(0.0, 0.5, 0.0, 1500);
    assert!(out_dtt.servo1_us > 1500); // Left aileron up
    assert!(out_dtt.servo4_us < 1500); // Right aileron down

    // Single Aileron V-Tail
    let mixer_vt = AirframeMixer::new(AirframeType::SingleAileronVTail);
    let out_vt = mixer_vt.mix(0.2, 0.3, 0.0, 1500);
    assert!(out_vt.servo1_us != 1500);
    assert!(out_vt.servo2_us != 1500);
    assert!(out_vt.servo3_us != 1500);

    // Dual Aileron V-Tail
    let mixer_dvt = AirframeMixer::new(AirframeType::DualAileronVTail);
    let out_dvt = mixer_dvt.mix(0.2, 0.3, 0.1, 1500);
    assert!(out_dvt.servo1_us != 1500);
    assert!(out_dvt.servo3_us != 1500);
    assert!(out_dvt.servo2_us != 1500);
    assert!(out_dvt.servo3_us != 1500);
}

#[test]
fn test_board_orientation_transforms() {
    use wingfc_core::math::{BoardOrientation, Vector3};

    let v = Vector3::new(1.0, 2.0, 3.0);

    // Default
    assert_eq!(
        BoardOrientation::Default.transform(v),
        Vector3::new(1.0, 2.0, 3.0)
    );

    // CW90: (y, -x, z)
    assert_eq!(
        BoardOrientation::CW90.transform(v),
        Vector3::new(2.0, -1.0, 3.0)
    );

    // CW180: (-x, -y, z)
    assert_eq!(
        BoardOrientation::CW180.transform(v),
        Vector3::new(-1.0, -2.0, 3.0)
    );

    // CW270: (-y, x, z)
    assert_eq!(
        BoardOrientation::CW270.transform(v),
        Vector3::new(-2.0, 1.0, 3.0)
    );

    // Flip: (x, -y, -z)
    assert_eq!(
        BoardOrientation::Flip.transform(v),
        Vector3::new(1.0, -2.0, -3.0)
    );

    // FlipCW90: (-y, -x, -z)
    assert_eq!(
        BoardOrientation::FlipCW90.transform(v),
        Vector3::new(-2.0, -1.0, -3.0)
    );

    // FlipCW180: (-x, y, -z)
    assert_eq!(
        BoardOrientation::FlipCW180.transform(v),
        Vector3::new(-1.0, 2.0, -3.0)
    );

    // FlipCW270: (y, x, -z)
    assert_eq!(
        BoardOrientation::FlipCW270.transform(v),
        Vector3::new(2.0, 1.0, -3.0)
    );
}

#[test]
fn test_in_flight_tuning_pots() {
    use wingfc_core::pid::{PidConfig, PidController};
    use wingfc_core::protocol::RcChannels;
    use wingfc_core::tuning::{InFlightTuning, TuneBank, TuneParameter, TuneSlot, MAX_TUNE_BANKS};

    let mut pitch_pid = PidController::new(PidConfig::default(), 0.002);
    let mut roll_pid = PidController::new(PidConfig::default(), 0.002);
    let mut yaw_pid = PidController::new(PidConfig::default(), 0.002);

    let mut rc = RcChannels::default();
    rc.channels[5] = 1000; // Slot A low
    rc.channels[6] = 2000; // Slot B high

    let mut banks = [TuneBank::default(); MAX_TUNE_BANKS];
    banks[0] = TuneBank::new(
        800,
        2200,
        TuneSlot::new(TuneParameter::PitchKp, 5, 0.1, 0.5),
        TuneSlot::new(TuneParameter::RollKd, 6, 0.01, 0.05),
    );

    let tuning = InFlightTuning::new(0xFF, 1, banks);

    tuning.update(&rc, &mut pitch_pid, &mut roll_pid, &mut yaw_pid);

    assert!((pitch_pid.config.kp - 0.1).abs() < 1e-4);
    assert!((roll_pid.config.kd - 0.05).abs() < 1e-4);

    // Now test mid-stick (1500 us)
    rc.channels[5] = 1500;
    tuning.update(&rc, &mut pitch_pid, &mut roll_pid, &mut yaw_pid);
    assert!((pitch_pid.config.kp - 0.3).abs() < 1e-4);
}

#[test]
fn test_in_flight_tuning_banks_and_persistence() {
    use wingfc_core::pid::{PidConfig, PidController};
    use wingfc_core::protocol::RcChannels;
    use wingfc_core::tuning::{InFlightTuning, TuneBank, TuneParameter, TuneSlot, MAX_TUNE_BANKS};

    let mut pitch_pid = PidController::new(PidConfig::default(), 0.002);
    let mut roll_pid = PidController::new(PidConfig::default(), 0.002);
    let mut yaw_pid = PidController::new(PidConfig::default(), 0.002);

    let mut banks = [TuneBank::default(); MAX_TUNE_BANKS];
    // Bank 1 (LOW 900..1300): Tune Kp
    banks[0] = TuneBank::new(
        900,
        1300,
        TuneSlot::new(TuneParameter::PitchKp, 5, 0.1, 0.5),
        TuneSlot::new(TuneParameter::RollKp, 6, 0.1, 0.5),
    );
    // Bank 2 (MID 1301..1699): Lock / Bypass
    banks[1] = TuneBank::new(
        1301,
        1699,
        TuneSlot::new(TuneParameter::None, 0, 0.0, 0.0),
        TuneSlot::new(TuneParameter::None, 0, 0.0, 0.0),
    );
    // Bank 3 (HIGH 1700..2100): Tune Kd & Yaw Kp
    banks[2] = TuneBank::new(
        1700,
        2100,
        TuneSlot::new(TuneParameter::PitchKd, 5, 0.01, 0.05),
        TuneSlot::new(TuneParameter::YawKp, 6, 0.2, 0.8),
    );

    // Selector channel on CH5 (idx 4)
    let tuning = InFlightTuning::new(4, 3, banks);

    let mut rc = RcChannels::default();

    // 1. Selector in Bank 1 (LOW): CH5 = 1000 us
    rc.channels[4] = 1000;
    rc.channels[5] = 2000; // CH6 max -> PitchKp = 0.5
    rc.channels[6] = 1000; // CH7 min -> RollKp = 0.1
    tuning.update(&rc, &mut pitch_pid, &mut roll_pid, &mut yaw_pid);

    assert!((pitch_pid.config.kp - 0.5).abs() < 1e-4);
    assert!((roll_pid.config.kp - 0.1).abs() < 1e-4);

    // 2. Switch to Bank 2 (MID / Lock): CH5 = 1500 us
    // Pots move (e.g. pilot knocks pot A to 1000 us)
    rc.channels[4] = 1500;
    rc.channels[5] = 1000;
    tuning.update(&rc, &mut pitch_pid, &mut roll_pid, &mut yaw_pid);

    // PitchKp and RollKp MUST remain locked at 0.5 and 0.1
    assert!((pitch_pid.config.kp - 0.5).abs() < 1e-4);
    assert!((roll_pid.config.kp - 0.1).abs() < 1e-4);

    // 3. Switch to Bank 3 (HIGH): CH5 = 1900 us
    rc.channels[4] = 1900;
    rc.channels[5] = 2000; // CH6 max -> PitchKd = 0.05
    rc.channels[6] = 1500; // CH7 mid -> YawKp = 0.5
    tuning.update(&rc, &mut pitch_pid, &mut roll_pid, &mut yaw_pid);

    assert!((pitch_pid.config.kd - 0.05).abs() < 1e-4);
    assert!((yaw_pid.config.kp - 0.5).abs() < 1e-4);
    // Crucial: PitchKp and RollKp must still persist untouched from Bank 1!
    assert!((pitch_pid.config.kp - 0.5).abs() < 1e-4);
    assert!((roll_pid.config.kp - 0.1).abs() < 1e-4);

    // 4. Return to Bank 2 (MID / Lock): CH5 = 1500 us
    rc.channels[4] = 1500;
    rc.channels[5] = 1200; // Wiggle pots
    rc.channels[6] = 1800;
    tuning.update(&rc, &mut pitch_pid, &mut roll_pid, &mut yaw_pid);

    // All dialed gains stay locked
    assert!((pitch_pid.config.kp - 0.5).abs() < 1e-4);
    assert!((roll_pid.config.kp - 0.1).abs() < 1e-4);
    assert!((pitch_pid.config.kd - 0.05).abs() < 1e-4);
    assert!((yaw_pid.config.kp - 0.5).abs() < 1e-4);
}

#[test]
fn test_compass_scaling_qmc5883l() {
    const SCALE_TO_UT: f32 = 100.0 / 3000.0;
    let raw_x: i16 = 300; // 0.1 Gauss = 10 uT
    let ut_x = (raw_x as f32) * SCALE_TO_UT;
    assert!((ut_x - 10.0).abs() < 1e-3);
}

#[test]
fn test_compass_scaling_hmc5883l() {
    const SCALE_TO_UT: f32 = 100.0 / 1090.0;
    let raw_x: i16 = 109; // 0.1 Gauss = 10 uT
    let ut_x = (raw_x as f32) * SCALE_TO_UT;
    assert!((ut_x - 10.0).abs() < 1e-2);
}

#[test]
fn test_mode_matrix_evaluation_and_priority() {
    use wingfc_core::mode::{FlightMode, ModeMatrix, ModeRange};

    let matrix = ModeMatrix::new([
        ModeRange::new(FlightMode::Angle, 5, 1300, 1700), // CH6: 1300..1700 = Angle
        ModeRange::new(FlightMode::Acro, 5, 1701, 2100),  // CH6: >1700 = Acro
    ]);

    let mut ch = [1500u16; 16];
    ch[5] = 1000; // CH6 low -> Default Acro
    assert_eq!(matrix.evaluate(&ch), FlightMode::Acro);

    ch[5] = 1500; // CH6 mid -> Angle
    assert_eq!(matrix.evaluate(&ch), FlightMode::Angle);
}

#[cfg(all(feature = "compass", feature = "baro", feature = "gps"))]
#[test]
fn test_mode_matrix_full_nav_priority() {
    use wingfc_core::mode::{FlightMode, ModeMatrix, ModeRange};

    let matrix = ModeMatrix::new([
        ModeRange::new(FlightMode::Angle, 5, 1300, 1700), // CH6: 1300..1700 = Angle
        ModeRange::new(FlightMode::HeadingHold, 5, 1701, 2100), // CH6: >1700 = HeadingHold
        ModeRange::new(FlightMode::AltHold, 6, 1700, 2100), // CH7: >1700 = AltHold
        ModeRange::new(FlightMode::Rth, 7, 1800, 2100),   // CH8: >1800 = Emergency RTH
    ]);

    let mut ch = [1500u16; 16];
    ch[5] = 1000;
    assert_eq!(matrix.evaluate(&ch), FlightMode::Acro);

    ch[5] = 1500;
    assert_eq!(matrix.evaluate(&ch), FlightMode::Angle);

    ch[5] = 1900;
    assert_eq!(matrix.evaluate(&ch), FlightMode::HeadingHold);

    ch[6] = 1900;
    assert_eq!(matrix.evaluate(&ch), FlightMode::AltHold);

    ch[7] = 2000;
    assert_eq!(matrix.evaluate(&ch), FlightMode::Rth);
}

#[cfg(feature = "compass")]
#[test]
fn test_cascaded_heading_hold() {
    use wingfc_core::mode::{HeadingHoldConfig, HeadingHoldController};

    let mut controller = HeadingHoldController::new(HeadingHoldConfig::default());

    let roll_cmd = controller.update(0.0, 0.0, 0.0);
    assert_eq!(roll_cmd, 0.0);

    let roll_cmd2 = controller.update(0.2, 0.0, 0.0);
    assert!(roll_cmd2 < 0.0);

    let roll_cmd3 = controller.update(-0.2, 0.0, 0.0);
    assert!(roll_cmd3 > 0.0);
}

#[cfg(feature = "baro")]
#[test]
fn test_cascaded_alt_hold() {
    use wingfc_core::mode::{AltHoldConfig, AltHoldController};

    let mut controller = AltHoldController::new(AltHoldConfig::default());

    let (pitch_cmd, thr_cmd) = controller.update(50.0, 0.0, 0.0, 1500);
    assert_eq!(pitch_cmd, 0.0);
    assert_eq!(thr_cmd, 1500);

    let (pitch_cmd2, _) = controller.update(45.0, 0.0, 0.0, 1500);
    assert!(pitch_cmd2 > 0.0);

    let (pitch_cmd3, _) = controller.update(55.0, 0.0, 0.0, 1500);
    assert!(pitch_cmd3 < 0.0);
}

#[test]
fn test_angle_mode_tilt_response() {
    use libm::{cosf, sinf};
    use wingfc_core::ahrs::VqfAHRS;
    use wingfc_core::mixer::{AirframeMixer, AirframeType};
    use wingfc_core::pid::{PidConfig, PidController};

    let dt = 0.002; // 500 Hz
    let angle_p_gain = 10.0;
    let max_roll_angle = 60.0 * core::f32::consts::PI / 180.0;
    let max_pitch_angle = 45.0 * core::f32::consts::PI / 180.0;

    let pid_cfg = PidConfig {
        kp: 1.0,
        ki: 0.5,
        kd: 0.08,
        kff: 0.04,
        max_integral: 0.4,
        dterm_lpf_hz: 40.0,
        ff_lpf_hz: 45.0,
    };

    let mut ahrs = VqfAHRS::new(0.5, 9.0, dt);
    let mut roll_pid = PidController::new(pid_cfg, dt);
    let mut pitch_pid = PidController::new(pid_cfg, dt);
    let mixer = AirframeMixer::new(AirframeType::Elevon);

    // Initial level state
    for _ in 0..500 {
        ahrs.update_6dof(Vector3::ZERO, Vector3::new(0.0, 0.0, 9.80665), dt);
    }
    let (p0, r0, _) = ahrs.euler_angles();
    println!("Level: pitch={}, roll={}", p0, r0);

    // Now aircraft is tilted right by 30 degrees (roll = +30 deg = +0.5236 rad)
    // In aircraft frame: gravity vector tilts to the left (-Y)
    let roll_angle = 30.0 * core::f32::consts::PI / 180.0;
    let accel_tilted = Vector3::new(0.0, -9.80665 * sinf(roll_angle), 9.80665 * cosf(roll_angle));

    // Simulate holding it tilted for 1 second (500 steps)
    let mut last_s1 = 1500;
    let mut last_s2 = 1500;
    for _ in 0..500 {
        ahrs.update_6dof(Vector3::ZERO, accel_tilted, dt);
        let (est_pitch, est_roll, _) = ahrs.euler_angles();

        // Sticks centered (0.0)
        let pitch_stick = 0.0;
        let roll_stick = 0.0;

        let target_pitch = pitch_stick * max_pitch_angle;
        let target_roll = roll_stick * max_roll_angle;

        let target_pitch_rate = (target_pitch - est_pitch) * angle_p_gain;
        let target_roll_rate = (target_roll - est_roll) * angle_p_gain;

        let roll_mix = roll_pid.update(target_roll_rate, 0.0, dt);
        let pitch_mix = pitch_pid.update(target_pitch_rate, 0.0, dt);

        let out = mixer.mix(pitch_mix, roll_mix, 0.0, 1000);
        last_s1 = out.servo1_us;
        last_s2 = out.servo2_us;
    }

    let (final_pitch, final_roll, _) = ahrs.euler_angles();
    println!("Tilted: est_pitch={}, est_roll={}", final_pitch, final_roll);
    println!("Servos: s1={}, s2={}", last_s1, last_s2);

    assert!(
        final_roll.abs() > 0.4,
        "Roll estimate should reflect 30 deg (~0.52 rad), got {}",
        final_roll
    );
    assert!(
        last_s1 != 1500,
        "Servo 1 should deflect to counter tilt, got {}",
        last_s1
    );
    assert!(
        last_s2 != 1500,
        "Servo 2 should deflect to counter tilt, got {}",
        last_s2
    );
}

#[test]
fn test_map_range_precomputed_and_servo_optimizations() {
    use wingfc_core::math::{map_range, map_range_precomputed};
    use wingfc_core::mixer::ServoConfig;

    // 1. Verify map_range_precomputed exactly matches map_range
    let in_min = 988.0;
    let in_max = 2012.0;
    let inv_range = 1.0 / (in_max - in_min);
    let out_min = -1.0;
    let out_range = 2.0;

    for raw in [988.0, 1000.0, 1500.0, 1800.0, 2012.0] {
        let standard = map_range(raw, in_min, in_max, out_min, out_min + out_range);
        let precomputed = map_range_precomputed(raw, in_min, inv_range, out_min, out_range);
        assert!(
            (standard - precomputed).abs() < 1e-5,
            "Mismatch at raw={}: standard={}, precomputed={}",
            raw,
            standard,
            precomputed
        );
    }

    // 2. Verify ServoConfig::apply across normal, reversed, subtrimmed and clamped configurations
    let configs = [
        ServoConfig {
            min_us: 1000,
            max_us: 2000,
            subtrim_us: 0,
            reversed: false,
        },
        ServoConfig {
            min_us: 1000,
            max_us: 2000,
            subtrim_us: 50,
            reversed: true,
        },
        ServoConfig {
            min_us: 900,
            max_us: 2100,
            subtrim_us: -25,
            reversed: false,
        },
        ServoConfig {
            min_us: 1100,
            max_us: 1900,
            subtrim_us: -40,
            reversed: true,
        },
    ];

    for cfg in &configs {
        for norm in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5] {
            // Compute expected via legacy map_range formula
            let mut expected = map_range(norm, -1.0, 1.0, cfg.min_us as f32, cfg.max_us as f32);
            if cfg.reversed {
                expected = (cfg.min_us + cfg.max_us) as f32 - expected;
            }
            expected += cfg.subtrim_us as f32;
            let expected_us =
                wingfc_core::math::clamp(expected, cfg.min_us as f32, cfg.max_us as f32) as u16;

            let actual_us = cfg.apply(norm);
            assert_eq!(
                actual_us, expected_us,
                "Mismatch for config {:?} at norm {}: actual={}, expected={}",
                cfg, norm, actual_us, expected_us
            );
        }
    }
}
