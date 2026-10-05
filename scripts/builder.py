#!/usr/bin/env python3
"""
WingFC-rs User-Friendly Build & Configuration Engine

Allows users to configure and build flight controller firmware using a friendly
YAML file (`wingfc.yaml`) without ever modifying Rust files or Cargo build flags.

Usage:
  ./scripts/build.sh                # Build firmware from wingfc.yaml
  ./scripts/build.sh check          # Validate config and check compilation
  ./scripts/build.sh test           # Run host test suite
  ./scripts/build.sh matrix         # Run full 8/16 variant build matrix
  ./scripts/build.sh init           # Reset/create default wingfc.yaml
  ./scripts/build.sh flash          # Flash built firmware using probe-rs
"""

import argparse
import glob
import hashlib
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is required. Please run: pip install pyyaml", file=sys.stderr)
    sys.exit(1)

TARGET = "thumbv7em-none-eabihf"
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = WORKSPACE_ROOT / "wingfc.yaml"
GENERATED_CONFIG_PATH = WORKSPACE_ROOT / "crates" / "wingfc-firmware" / "src" / "config.rs"
DIST_DIR = WORKSPACE_ROOT / "dist"

FLASH_TOTAL_BYTES = 1024 * 1024  # 1 MB Flash
RAM_TOTAL_BYTES = 256 * 1024     # 256 KB RAM

# Statutory Open Category Altitude Limit in meters Above Ground Level (AGL)
# FAA (14 CFR 107.51), EASA (EU 2019/947 UAS.OPEN.010), UK CAA (CAP 722): 120m (400 ft)
LEGAL_MAX_CEILING_AGL_M = 120.0

# Hardware PWM timer limits (nRF52840 1MHz clock with 15-bit counter)
MIN_PWM_HZ = 50
MAX_PWM_HZ = 400


def load_config(config_path):
    """Loads and parses the YAML aircraft configuration file."""
    if not config_path.exists():
        print(f"Error: Configuration file '{config_path}' does not exist.")
        print("Run './scripts/build.sh init' to generate a default configuration.")
        sys.exit(1)

    try:
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"Error parsing YAML file '{config_path}': {e}")
        sys.exit(1)

    if not isinstance(cfg, dict):
        print("Error: Invalid configuration format (root must be a YAML mapping).")
        sys.exit(1)

    return cfg


def validate_config(cfg):
    """Validates configuration parameters and returns sanitized feature flags and values."""
    errors = []
    advisories = []

    # 1. Board & Profile Target
    board = str(cfg.get("board", "xiao-sense")).strip().lower()
    valid_boards = ["xiao-sense", "xiao-sense-plus"]
    if board not in valid_boards:
        errors.append(f"Invalid board '{board}'. Must be one of: {valid_boards}")

    profile = str(cfg.get("profile", "ultralight")).strip().lower()
    valid_profiles = ["ultralight", "gps"]
    if profile not in valid_profiles:
        errors.append(f"Invalid profile '{profile}'. Must be one of: {valid_profiles}")

    valid_orientations = {
        "default": "BoardOrientation::Default",
        "cw90": "BoardOrientation::CW90",
        "cw180": "BoardOrientation::CW180",
        "cw270": "BoardOrientation::CW270",
        "flip": "BoardOrientation::Flip",
        "flip_cw90": "BoardOrientation::FlipCW90",
        "flip_cw180": "BoardOrientation::FlipCW180",
        "flip_cw270": "BoardOrientation::FlipCW270",
    }
    raw_orientation = str(cfg.get("orientation", "default")).strip().lower()
    if raw_orientation not in valid_orientations:
        errors.append(f"Invalid orientation '{raw_orientation}'. Supported: {list(valid_orientations.keys())}")
    orientation_rust = valid_orientations.get(raw_orientation, "BoardOrientation::Default")

    valid_airframes = {
        "elevon": "AirframeType::Elevon",
        "elevon_diff_thrust": "AirframeType::ElevonWithDiffThrust",
        "single_aileron_t_tail": "AirframeType::SingleAileronTTail",
        "dual_aileron_t_tail": "AirframeType::DualAileronTTail",
        "single_aileron_v_tail": "AirframeType::SingleAileronVTail",
        "dual_aileron_v_tail": "AirframeType::DualAileronVTail",
    }
    raw_airframe = str(cfg.get("airframe", "elevon")).strip().lower()
    if raw_airframe not in valid_airframes:
        errors.append(f"Invalid airframe '{raw_airframe}'. Supported: {list(valid_airframes.keys())}")
    airframe_rust = valid_airframes.get(raw_airframe, "AirframeType::Elevon")

    # Loop Rate & IMU ODR Profile
    valid_loop_rates = {
        "standard": {
            "profile": "LoopRateProfile::Standard416Hz",
            "hz": 416,
            "nominal_dt": 1.0 / 416.0,
            "xl_reg": 0x64,  # 416 Hz ODR, +/-16g (0x60 | 0x04)
            "g_reg": 0x6C,   # 416 Hz ODR, +/-2000 dps (0x60 | 0x0C)
            "name": "Standard (416 Hz)",
            "decimation": 1,
        },
        "high": {
            "profile": "LoopRateProfile::High833Hz",
            "hz": 833,
            "nominal_dt": 1.0 / 833.0,
            "xl_reg": 0x74,  # 833 Hz ODR, +/-16g (0x70 | 0x04)
            "g_reg": 0x7C,   # 833 Hz ODR, +/-2000 dps (0x70 | 0x0C)
            "name": "High-Speed (833 Hz)",
            "decimation": 1,
        },
        "multirate": {
            "profile": "LoopRateProfile::MultiRate208Hz",
            "hz": 208,
            "nominal_dt": 0.0048,  # 8 samples @ 1666 Hz
            "xl_reg": 0x84,  # 1.66 kHz ODR, +/-16g (0x80 | 0x04)
            "g_reg": 0x8C,   # 1.66 kHz ODR, +/-2000 dps (0x80 | 0x0C)
            "name": "Multi-Rate (1666 Hz -> 208.33 Hz, 8:1 boxcar)",
            "decimation": 8,
        },
    }
    raw_loop_rate = str(cfg.get("loop_rate", "standard")).strip().lower()
    if raw_loop_rate not in valid_loop_rates:
        errors.append(f"Invalid loop_rate '{raw_loop_rate}'. Supported: {list(valid_loop_rates.keys())}")
    loop_rate_info = valid_loop_rates.get(raw_loop_rate, valid_loop_rates["standard"])

    # 2. Receiver Protocol

    rc_cfg = cfg.get("receiver", {})
    proto = str(rc_cfg.get("protocol", "crsf")).strip().lower()
    if proto not in ["crsf", "ibus"]:
        errors.append(f"Invalid receiver.protocol '{proto}'. Must be 'crsf' or 'ibus'.")

    failsafe_ms = rc_cfg.get("failsafe_timeout_ms", 500)
    if not isinstance(failsafe_ms, (int, float)) or failsafe_ms < 50 or failsafe_ms > 5000:
        errors.append(f"receiver.failsafe_timeout_ms ({failsafe_ms}) must be between 50 and 5000 ms.")


    # Receiver Channel Mapping
    ch_cfg = rc_cfg.get("channels", {})
    def parse_rc_channel(name, default_ch):
        val = int(ch_cfg.get(name, default_ch))
        if val < 1 or val > 16:
            errors.append(f"receiver.channels.{name} ({val}) out of bounds [1, 16]")
        return val - 1 # 0-indexed for Rust arrays

    ch_roll_idx = parse_rc_channel("roll", 1)
    ch_pitch_idx = parse_rc_channel("pitch", 2)
    ch_throttle_idx = parse_rc_channel("throttle", 3)
    ch_yaw_idx = parse_rc_channel("yaw", 4)
    ch_arm_idx = parse_rc_channel("arm", 5)
    ch_mode_idx = parse_rc_channel("mode", 6)

    # Safety Interlocks & Thresholds
    safety_cfg = cfg.get("safety", {})
    arm_switch_threshold_us = int(safety_cfg.get("arm_switch_threshold_us", 1800))
    throttle_zero_threshold_us = int(safety_cfg.get("throttle_zero_threshold_us", 1050))

    if arm_switch_threshold_us < 1200 or arm_switch_threshold_us > 1900:
        errors.append("safety.arm_switch_threshold_us must be between 1200 and 1900 us.")
    elif arm_switch_threshold_us < 1400:
        advisories.append(
            f"SAFETY WARNING: arm_switch_threshold_us ({arm_switch_threshold_us} us) is configured below 1400 us!\n"
            f"    * This creates severe risk of accidental arming on 2-position or 3-position switches.\n"
            f"    * Recommended threshold is 1800 us."
        )

    if throttle_zero_threshold_us < 1000 or throttle_zero_threshold_us > 1200:
        errors.append("safety.throttle_zero_threshold_us must be between 1000 and 1200 us.")
    elif throttle_zero_threshold_us > 1080:
        advisories.append(
            f"CRITICAL SAFETY WARNING: throttle_zero_threshold_us ({throttle_zero_threshold_us} us) is set above 1080 us!\n"
            f"    * Permitting arming at elevated throttle stick positions poses extreme risk of violent propeller spin-up upon arming!\n"
            f"    * High throttle arming causes instant motor engagement, leading to catastrophic laceration injuries or runaway aircraft.\n"
            f"    * Keep strictly at 1050 us unless your transmitter idle trim requires small adjustments."
        )

    # 3. Servo Actuator Configuration
    servos_cfg = cfg.get("servos", {})
    servo_pwm_hz = int(servos_cfg.get("pwm_frequency_hz", 50))
    if servo_pwm_hz < MIN_PWM_HZ or servo_pwm_hz > MAX_PWM_HZ:
        errors.append(
            f"servos.pwm_frequency_hz ({servo_pwm_hz} Hz) out of bounds! "
            f"Hardware supported range is {MIN_PWM_HZ} Hz to {MAX_PWM_HZ} Hz. "
            f"(Frequencies >400 Hz do not allow enough low off-time for standard 1000..2000 us servo pulses)."
        )
    elif servo_pwm_hz > 60:
        advisories.append(
            f"HARDWARE WARNING: Servo refresh frequency is set to {servo_pwm_hz} Hz!\n"
            f"    * Standard analog servos (e.g., SG90, MG90S, 9g micro servos) CANNOT exceed 50-60 Hz.\n"
            f"      Driving analog servos above 60 Hz causes excessive heating, chattering, and rapid motor burnout.\n"
            f"    * Ensure your elevon servos are DIGITAL models that explicitly support {servo_pwm_hz} Hz refresh."
        )


    # Per-servo channel configurations (servo1, servo2, servo4, servo5, servo6)
    def parse_servo(s_name, def_min=1000, def_max=2000, def_subtrim=0, def_rev=False):
        sc = servos_cfg.get(s_name, {})
        s_min = int(sc.get("min_us", def_min))
        s_max = int(sc.get("max_us", def_max))
        s_subtrim = int(sc.get("subtrim_us", def_subtrim))
        s_rev = bool(sc.get("reversed", def_rev))
        if s_min < 750 or s_min > 1500:
            errors.append(f"servos.{s_name}.min_us ({s_min}) out of bounds [750, 1500]")
        if s_max < 1500 or s_max > 2250:
            errors.append(f"servos.{s_name}.max_us ({s_max}) out of bounds [1500, 2250]")
        if s_subtrim < -500 or s_subtrim > 500:
            errors.append(f"servos.{s_name}.subtrim_us ({s_subtrim}) out of bounds [-500, 500]")
        return f"ServoConfig {{ min_us: {s_min}, max_us: {s_max}, subtrim_us: {s_subtrim}, reversed: {str(s_rev).lower()} }}"

    servo1_rust = parse_servo("servo1")
    servo2_rust = parse_servo("servo2")
    servo3_rust = parse_servo("servo3")
    servo4_rust = parse_servo("servo4")
    servo5_rust = parse_servo("servo5")
    servo6_rust = parse_servo("servo6")
    servo7_rust = parse_servo("servo7")
    servo8_rust = parse_servo("servo8")

    # In-Flight Dynamic PID Tuning (Banks & Slots)
    param_map = {
        "none": "TuneParameter::None",
        "pitch_kp": "TuneParameter::PitchKp",
        "roll_kp": "TuneParameter::RollKp",
        "pitch_ki": "TuneParameter::PitchKi",
        "roll_ki": "TuneParameter::RollKi",
        "pitch_kd": "TuneParameter::PitchKd",
        "roll_kd": "TuneParameter::RollKd",
        "pitch_kff": "TuneParameter::PitchKff",
        "roll_kff": "TuneParameter::RollKff",
        "yaw_kp": "TuneParameter::YawKp",
        "yaw_ki": "TuneParameter::YawKi",
        "yaw_kd": "TuneParameter::YawKd",
        "yaw_kff": "TuneParameter::YawKff",
    }
    ift_cfg = cfg.get("in_flight_tuning", {})
    ift_enabled = bool(ift_cfg.get("enabled", False))

    def parse_slot_dict(sc, prefix, def_ch=6):
        param = str(sc.get("parameter", "none")).strip().lower()
        if param not in param_map:
            errors.append(f"{prefix}.parameter '{param}' invalid. Valid: {list(param_map.keys())}")
        param_rust = param_map.get(param, "TuneParameter::None")
        ch = int(sc.get("channel", def_ch))
        if ch < 1 or ch > 16:
            errors.append(f"{prefix}.channel ({ch}) out of bounds [1, 16]")
        min_v = float(sc.get("min_val", 0.0))
        max_v = float(sc.get("max_val", 1.0))
        ch_idx = max(0, ch - 1)
        return f"TuneSlot {{ parameter: {param_rust}, channel_idx: {ch_idx}, min_val: {min_v:.4f}, max_val: {max_v:.4f} }}"

    raw_banks = ift_cfg.get("banks", [])
    selector_ch = ift_cfg.get("selector_channel", None)
    if selector_ch is not None:
        try:
            sel_ch_int = int(selector_ch)
            if sel_ch_int < 1 or sel_ch_int > 16:
                errors.append(f"in_flight_tuning.selector_channel ({sel_ch_int}) out of bounds [1, 16]")
                selector_ch_idx = 0xFF
            else:
                selector_ch_idx = sel_ch_int - 1
        except (ValueError, TypeError):
            errors.append(f"in_flight_tuning.selector_channel '{selector_ch}' must be an integer between 1 and 16")
            selector_ch_idx = 0xFF
    else:
        selector_ch_idx = 0xFF

    parsed_banks = []
    if raw_banks:
        if len(raw_banks) > 6:
            errors.append(f"in_flight_tuning: maximum 6 banks supported, found {len(raw_banks)}")
        for idx, b in enumerate(raw_banks[:6]):
            prefix = f"in_flight_tuning.banks[{idx}]"
            sw_range = b.get("switch_range", [800, 2200])
            if not isinstance(sw_range, (list, tuple)) or len(sw_range) != 2:
                errors.append(f"{prefix}.switch_range must be a [min_us, max_us] pair, got {sw_range}")
                r_min, r_max = 800, 2200
            else:
                r_min = int(sw_range[0])
                r_max = int(sw_range[1])
                if r_min < 800 or r_max > 2200 or r_min > r_max:
                    errors.append(f"{prefix}.switch_range [{r_min}, {r_max}] invalid. Must satisfy 800 <= min <= max <= 2200.")
            slot_a_rust = parse_slot_dict(b.get("slot_a", {}), f"{prefix}.slot_a", def_ch=6)
            slot_b_rust = parse_slot_dict(b.get("slot_b", {}), f"{prefix}.slot_b", def_ch=7)
            parsed_banks.append(f"TuneBank {{\n            range_min_us: {r_min},\n            range_max_us: {r_max},\n            slot_a: {slot_a_rust},\n            slot_b: {slot_b_rust},\n        }}")
    else:
        # Legacy fallback: single bank from top-level slot_a and slot_b
        slot_a_rust = parse_slot_dict(ift_cfg.get("slot_a", {}), "in_flight_tuning.slot_a", def_ch=6)
        slot_b_rust = parse_slot_dict(ift_cfg.get("slot_b", {}), "in_flight_tuning.slot_b", def_ch=7)
        parsed_banks.append(f"TuneBank {{\n            range_min_us: 800,\n            range_max_us: 2200,\n            slot_a: {slot_a_rust},\n            slot_b: {slot_b_rust},\n        }}")

    num_banks = len(parsed_banks)
    # Pad out to MAX_TUNE_BANKS (6) with const DEFAULT
    while len(parsed_banks) < 6:
        parsed_banks.append("TuneBank::DEFAULT")

    banks_rust = "[\n        " + ",\n        ".join(parsed_banks) + ",\n    ]"
    # 4. Motor ESC Protocol & PWM Frequency
    motor_cfg = cfg.get("motor", {})
    motor_proto = str(motor_cfg.get("protocol", "pwm")).strip().lower()
    if motor_proto not in ["pwm", "dshot"]:
        errors.append(f"Invalid motor.protocol '{motor_proto}'. Must be 'pwm' or 'dshot'.")

    diff_thrust = float(motor_cfg.get("differential_thrust", 0.15))
    if diff_thrust < 0.0 or diff_thrust > 1.0:
        errors.append(f"motor.differential_thrust ({diff_thrust}) out of bounds! Must be between 0.0 and 1.0.")

    motor_pwm_hz = int(motor_cfg.get("pwm_frequency_hz", 400))
    if motor_proto == "pwm":
        if motor_pwm_hz < MIN_PWM_HZ or motor_pwm_hz > MAX_PWM_HZ:
            errors.append(
                f"motor.pwm_frequency_hz ({motor_pwm_hz} Hz) out of bounds! "
                f"Hardware supported range is {MIN_PWM_HZ} Hz to {MAX_PWM_HZ} Hz. "
                f"(Frequencies >400 Hz do not allow enough low off-time for standard 1000..2000 us throttle pulses)."
            )
        elif motor_pwm_hz > 100:
            advisories.append(
                f"HARDWARE NOTICE: Analog ESC PWM is configured at {motor_pwm_hz} Hz.\n"
                f"    * High-speed multirotor ESCs (BLHeli_S/32, AM32) support 400 Hz natively.\n"
                f"    * Legacy or basic fixed-wing airplane ESCs may only accept 50 Hz or 60 Hz.\n"
                f"      If your ESC fails to arm, beeps continually, or stutters, reduce motor.pwm_frequency_hz to 50 Hz."
            )

    # 5. Navigation & Altitude Limits
    nav_cfg = cfg.get("navigation", {})
    nav_enabled = bool(nav_cfg.get("enabled", False))
    baro_raw = nav_cfg.get("barometer", False)
    if isinstance(baro_raw, dict):
        baro_enabled = bool(baro_raw.get("enabled", False)) or nav_enabled
        baro_model = str(baro_raw.get("model", "dps310")).lower().strip()
        baro_i2c_addr = int(baro_raw.get("i2c_address", 0x77))
    else:
        baro_enabled = bool(baro_raw) or nav_enabled
        baro_model = str(nav_cfg.get("barometer_model", "dps310")).lower().strip()
        baro_i2c_addr = int(nav_cfg.get("barometer_i2c_address", 0x77))

    if baro_enabled and baro_model not in ["dps310", "bmp280"]:
        errors.append(f"Invalid barometer model '{baro_model}'. Supported models: 'dps310', 'bmp280'.")

    compass_raw = nav_cfg.get("compass", False)
    compass_declination_deg = 0.0
    compass_hard_iron = [0.0, 0.0, 0.0]
    compass_soft_iron = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    if isinstance(compass_raw, dict):
        compass_enabled = bool(compass_raw.get("enabled", False)) or nav_enabled
        compass_model = str(compass_raw.get("model", "qmc5883l")).lower().strip()
        compass_declination_deg = float(compass_raw.get("declination_deg", 0.0))
        hi = compass_raw.get("hard_iron", [0.0, 0.0, 0.0])
        if isinstance(hi, list) and len(hi) == 3:
            compass_hard_iron = [float(x) for x in hi]
        si = compass_raw.get("soft_iron", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        if isinstance(si, list) and len(si) == 3 and all(isinstance(r, list) and len(r) == 3 for r in si):
            compass_soft_iron = [[float(val) for val in row] for row in si]
    else:
        compass_enabled = bool(compass_raw) or nav_enabled
        compass_model = str(nav_cfg.get("compass_model", "qmc5883l")).lower().strip()

    if compass_enabled and compass_model not in ["qmc5883l", "hmc5883l"]:
        errors.append(f"Invalid compass model '{compass_model}'. Supported models: 'qmc5883l', 'hmc5883l'.")
    gps_enabled = bool(nav_cfg.get("gps", False)) or (nav_enabled and profile == "gps")

    if profile == "ultralight" and gps_enabled:
        errors.append(
            "GPS navigation cannot be enabled under 'profile: ultralight'. "
            "To enable GPS navigation and Return-to-Home, set 'profile: gps' and 'board: xiao-sense-plus'."
        )

    rth_cfg = nav_cfg.get("rth", {})
    rth_min_sats = int(rth_cfg.get("min_satellites", 6))
    rth_max_hdop = float(rth_cfg.get("max_hdop", 2.5))
    rth_safe_alt = float(rth_cfg.get("safe_altitude_agl_m", 30.0))

    if gps_enabled and board != "xiao-sense-plus":
        errors.append(
            f"GPS navigation (profile: gps) requires board: 'xiao-sense-plus'. "
            f"The standard '{board}' board lacks the secondary UART (UARTE1) broken out on pins D9/D10."
        )

    hard_deck_m = float(nav_cfg.get("hard_deck_agl_m", 15.0))
    max_ceiling_m = float(nav_cfg.get("max_ceiling_agl_m", 120.0))

    if hard_deck_m < 0.0 or hard_deck_m > 300.0:
        errors.append("navigation.hard_deck_agl_m must be between 0.0 and 300.0 meters AGL.")
    if max_ceiling_m <= hard_deck_m:
        errors.append(f"navigation.max_ceiling_agl_m ({max_ceiling_m}m) must be strictly greater than hard_deck_agl_m ({hard_deck_m}m).")

    # Legality warning check on maximum ceiling
    if max_ceiling_m > LEGAL_MAX_CEILING_AGL_M:
        diff_m = max_ceiling_m - LEGAL_MAX_CEILING_AGL_M
        diff_ft = diff_m * 3.28084
        advisories.append(
            f"LEGALITY WARNING: Configured altitude ceiling ({max_ceiling_m:.1f} m / {max_ceiling_m * 3.28084:.0f} ft AGL) EXCEEDS the standard "
            f"statutory limit of {LEGAL_MAX_CEILING_AGL_M:.1f} m (400 ft) AGL by +{diff_m:.1f} m (+{diff_ft:.0f} ft)!\n"
            f"    * FAA 14 CFR § 107.51(b) / 49 U.S.C. § 44809 (United States): Maximum sUAS altitude is 400 ft AGL without an approved Part 107.200 certificate/waiver.\n"
            f"    * EASA Regulation (EU) 2019/947 UAS.OPEN.010 (European Union): Flight height is strictly limited to 120 m AGL in Open category.\n"
            f"    * UK CAA CAP 722 / Air Navigation Order (United Kingdom): Maximum flight height is 120 m (400 ft) AGL.\n"
            f"    * CASA (Australia) / Transport Canada (CARs Part IX): 400 ft (120 m) AGL maximum ceiling.\n"
            f"    --> Operating above 120 m (400 ft) AGL without formal civil aviation authorization is ILLEGAL, creates extreme collision hazards for manned aviation, and exposes the operator to civil fines and criminal prosecution."
        )

    # INAV-style Flight Mode Matrix
    raw_modes = cfg.get("modes", [])
    mode_ranges = []
    if raw_modes and isinstance(raw_modes, list):
        for entry in raw_modes:
            m_name = str(entry.get("mode", "angle")).lower().replace("-", "_").replace(" ", "_")
            if m_name in ("heading_hold", "headinghold"):
                if not (nav_enabled or compass_enabled):
                    errors.append("Mode 'heading_hold' configured in modes matrix, but navigation.compass.enabled is false.")
                variant = "FlightMode::HeadingHold"
            elif m_name in ("alt_hold", "althold"):
                if not (nav_enabled or baro_enabled):
                    errors.append("Mode 'alt_hold' configured in modes matrix, but navigation.barometer.enabled is false.")
                variant = "FlightMode::AltHold"
            elif m_name == "rth":
                if not (nav_enabled or gps_enabled):
                    errors.append("Mode 'rth' configured in modes matrix, but navigation.gps.enabled is false.")
                variant = "FlightMode::Rth"
            elif m_name == "acro":
                variant = "FlightMode::Acro"
            else:
                variant = "FlightMode::Angle"

            c_idx = max(0, min(15, int(entry.get("channel", 6)) - 1))
            mn_us = int(entry.get("min_us", 1300))
            mx_us = int(entry.get("max_us", 1700))
            mode_ranges.append(f"ModeRange::new({variant}, {c_idx}, {mn_us}, {mx_us})")
    if not mode_ranges:
        # Default fallback: 2-position switch on CH6: Angle in [1000, 1500], Acro in [1501, 2000]
        mode_ranges = [
            f"ModeRange::new(FlightMode::Angle, {ch_mode_idx}, 1000, 1500)",
            f"ModeRange::new(FlightMode::Acro, {ch_mode_idx}, 1501, 2000)",
        ]
    ranges_joined = ", ".join(mode_ranges)
    mode_matrix_rust = f"ModeMatrix::new([{ranges_joined}])"
    mode_count = len(mode_ranges)

    # 6. Tuning & Rates
    tuning_cfg = cfg.get("tuning", {})
    acro = tuning_cfg.get("acro_rates", {})
    roll_dps = float(acro.get("roll_dps", 600.0))
    pitch_dps = float(acro.get("pitch_dps", 200.0))
    yaw_dps = float(acro.get("yaw_dps", 100.0))

    if roll_dps <= 0 or roll_dps > 2000.0:
        errors.append("tuning.acro_rates.roll_dps must be between 10.0 and 2000.0 deg/s.")
    if pitch_dps <= 0 or pitch_dps > 2000.0:
        errors.append("tuning.acro_rates.pitch_dps must be between 10.0 and 2000.0 deg/s.")
    if yaw_dps <= 0 or yaw_dps > 2000.0:
        errors.append("tuning.acro_rates.yaw_dps must be between 10.0 and 2000.0 deg/s.")

    angle_lim = tuning_cfg.get("angle_limits", {})
    max_roll_deg = float(angle_lim.get("max_roll_deg", 60.0))
    max_pitch_deg = float(angle_lim.get("max_pitch_deg", 45.0))
    if max_roll_deg < 10.0 or max_roll_deg > 85.0:
        errors.append("tuning.angle_limits.max_roll_deg must be between 10.0 and 85.0 deg.")
    if max_pitch_deg < 10.0 or max_pitch_deg > 85.0:
        errors.append("tuning.angle_limits.max_pitch_deg must be between 10.0 and 85.0 deg.")
    angle_p_gain = float(angle_lim.get("angle_p_gain", 4.0))
    if angle_p_gain < 0.5 or angle_p_gain > 20.0:
        errors.append("tuning.angle_limits.angle_p_gain must be between 0.5 and 20.0 s^-1.")

    filters = tuning_cfg.get("filters", {})
    gyro_lpf = float(filters.get("gyro_lpf_hz", 90.0))
    dterm_lpf = float(filters.get("dterm_lpf_hz", 40.0))
    ff_lpf = float(filters.get("feedforward_lpf_hz", 45.0))
    if gyro_lpf < 10.0 or gyro_lpf > 250.0:
        errors.append("tuning.filters.gyro_lpf_hz must be between 10.0 and 250.0 Hz.")

    # 7. Flashing Configuration
    flash_cfg = cfg.get("flashing", {})
    flash_method = str(flash_cfg.get("method", "auto")).lower()
    valid_methods = ["auto", "nrfjprog", "uf2", "dfu", "probe-rs", "probe-rs-hex", "probe-rs-run"]
    if flash_method not in valid_methods:
        errors.append(
            f"flashing.method '{flash_method}' is invalid. Supported methods: {', '.join(valid_methods)}"
        )
    flash_port = flash_cfg.get("port", "auto")
    flash_mount = flash_cfg.get("mount", "auto")

    pid_cfg = tuning_cfg.get("rate_pid", {})
    kp = float(pid_cfg.get("kp", 0.15))
    ki = float(pid_cfg.get("ki", 0.05))
    kd = float(pid_cfg.get("kd", 0.008))
    kff = float(pid_cfg.get("kff", 0.04))
    max_integral = float(pid_cfg.get("max_integral", 0.40))
    ahrs_beta = float(tuning_cfg.get("ahrs_beta", 0.04))

    if errors:
        print("=" * 70)
        print("CONFIGURATION VALIDATION ERRORS IN wingfc.yaml:")
        print("=" * 70)
        for err in errors:
            print(f" - {err}")
        print("=" * 70)
        sys.exit(1)

    # Determine required cargo features (granular: baro, compass, gps)
    features = [proto]
    if motor_proto == "dshot":
        features.append("dshot")

    if baro_enabled:
        features.append("baro")
    if compass_enabled:
        features.append("compass")
    if gps_enabled:
        features.append("gps")

    variant_parts = [proto, motor_proto]
    if profile == "gps" or gps_enabled:
        variant_parts.append("gps")
    elif baro_enabled and compass_enabled:
        variant_parts.append("nav")
    elif baro_enabled:
        variant_parts.append("baro")
    elif compass_enabled:
        variant_parts.append("compass")
    else:
        variant_parts.append("ultralight")

    variant_name = "-".join(variant_parts)

    if raw_loop_rate == "high":
        if nav_enabled or gps_enabled:
            advisories.append("HIGH-SPEED LOOP (833 Hz) WITH NAVIGATION: Autonomous waypoint math and Kalman filters will share a tighter 1.2ms cycle. Ensure CPU load remains within bounds.")
        if servo_pwm_hz <= 60:
            advisories.append("HIGH-SPEED LOOP (833 Hz) WITH STANDARD SERVOS (50-60 Hz): Actuator update latency is limited by physical servo PWM frequency. Consider digital servos (200-400 Hz) or DShot ESCs to fully leverage 833 Hz gyro feedback.")

    return {
        "board": board,
        "profile": profile,
        "orientation_rust": orientation_rust,
        "airframe_rust": airframe_rust,
        "loop_rate_name": loop_rate_info["name"],
        "loop_rate_profile": loop_rate_info["profile"],
        "nominal_loop_hz": loop_rate_info["hz"],
        "nominal_dt": loop_rate_info["nominal_dt"],
        "imu_xl_reg": loop_rate_info["xl_reg"],
        "imu_g_reg": loop_rate_info["g_reg"],
        "imu_decimation": loop_rate_info["decimation"],
        "protocol": proto,

        "failsafe_ms": int(failsafe_ms),
        "ch_roll_idx": ch_roll_idx,
        "ch_pitch_idx": ch_pitch_idx,
        "ch_throttle_idx": ch_throttle_idx,
        "ch_yaw_idx": ch_yaw_idx,
        "ch_arm_idx": ch_arm_idx,
        "ch_mode_idx": ch_mode_idx,
        "mode_matrix_rust": mode_matrix_rust,
        "mode_count": mode_count,
        "arm_switch_threshold_us": arm_switch_threshold_us,
        "throttle_zero_threshold_us": throttle_zero_threshold_us,
        "servo_pwm_hz": servo_pwm_hz,
        "servo1_rust": servo1_rust,
        "servo2_rust": servo2_rust,
        "servo3_rust": servo3_rust,
        "servo4_rust": servo4_rust,
        "servo5_rust": servo5_rust,
        "servo6_rust": servo6_rust,
        "servo7_rust": servo7_rust,
        "servo8_rust": servo8_rust,
        "ift_enabled": "true" if ift_enabled else "false",
        "ift_selector_idx": f"0x{selector_ch_idx:02X}" if selector_ch_idx == 0xFF else str(selector_ch_idx),
        "ift_num_banks": str(num_banks),
        "ift_banks_rust": banks_rust,
        "slot_a_rust": slot_a_rust,
        "slot_b_rust": slot_b_rust,
        "motor_proto": motor_proto,
        "motor_pwm_hz": motor_pwm_hz,
        "diff_thrust": diff_thrust,
        "nav_enabled": nav_enabled,
        "baro_enabled": baro_enabled,
        "baro_model": baro_model,
        "baro_model_rust": "BaroModel::" + ("Dps310" if baro_model == "dps310" else "Bmp280"),
        "baro_i2c_addr": f"0x{baro_i2c_addr:02X}",
        "compass_enabled": compass_enabled,
        "compass_model": compass_model,
        "compass_model_rust": "CompassModel::" + ("Qmc5883l" if compass_model == "qmc5883l" else "Hmc5883l"),
        "compass_declination_rad": f"{(compass_declination_deg * 3.141592653589793 / 180.0):.6f}",
        "compass_hard_iron_rust": f"Vector3::new({compass_hard_iron[0]:.4f}, {compass_hard_iron[1]:.4f}, {compass_hard_iron[2]:.4f})",
        "compass_soft_iron_rust": f"[\n    [{compass_soft_iron[0][0]:.4f}, {compass_soft_iron[0][1]:.4f}, {compass_soft_iron[0][2]:.4f}],\n    [{compass_soft_iron[1][0]:.4f}, {compass_soft_iron[1][1]:.4f}, {compass_soft_iron[1][2]:.4f}],\n    [{compass_soft_iron[2][0]:.4f}, {compass_soft_iron[2][1]:.4f}, {compass_soft_iron[2][2]:.4f}],\n]",
        "gps_enabled": gps_enabled,
        "rth_min_sats": rth_min_sats,
        "rth_max_hdop": rth_max_hdop,
        "rth_safe_alt": rth_safe_alt,
        "hard_deck_m": hard_deck_m,
        "max_ceiling_m": max_ceiling_m,
        "advisories": advisories,
        "features": features,
        "features_str": ",".join(features),
        "variant_name": variant_name,
        "roll_dps": roll_dps,
        "pitch_dps": pitch_dps,
        "yaw_dps": yaw_dps,
        "max_roll_deg": max_roll_deg,
        "max_pitch_deg": max_pitch_deg,
        "angle_p_gain": angle_p_gain,
        "gyro_lpf": gyro_lpf,
        "dterm_lpf": dterm_lpf,
        "ff_lpf": ff_lpf,
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "kff": kff,
        "max_integral": max_integral,
        "ahrs_beta": ahrs_beta,
        "flash_method": flash_method,
        "flash_port": flash_port,
        "flash_mount": flash_mount,
    }


def generate_rust_config(cfg_vals):
    """Generates crates/wingfc-firmware/src/config.rs from validated parameters."""
    code = f"""//! User aircraft configuration parameters for `wingfc-firmware`.
//!
//! Generated automatically from `wingfc.yaml` by the build system.
//! Users configure their aircraft via `wingfc.yaml` without editing Rust source code.

use embassy_time::Duration;
use wingfc_core::math::{{BoardOrientation, Vector3, DEG_TO_RAD}};
use wingfc_core::mixer::{{AirframeType, ServoConfig}};
use wingfc_core::tuning::{{InFlightTuning, TuneBank, TuneParameter, TuneSlot}};
use wingfc_core::pid::PidConfig;
use wingfc_core::mode::{{FlightMode, ModeMatrix, ModeRange}};

/// Board mounting orientation
pub const BOARD_ORIENTATION: BoardOrientation = {cfg_vals['orientation_rust']};

/// Configured airframe mixer topology
pub const AIRFRAME_TYPE: AirframeType = {cfg_vals['airframe_rust']};

/// Selectable flight loop timing profiles
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopRateProfile {{
    Standard416Hz,
    High833Hz,
    MultiRate208Hz,
}}

/// Configured flight loop rate profile
pub const LOOP_RATE_PROFILE: LoopRateProfile = {cfg_vals['loop_rate_profile']};

/// Nominal flight loop frequency in Hz
pub const NOMINAL_LOOP_HZ: u32 = {cfg_vals['nominal_loop_hz']};

/// Nominal flight loop time-step in seconds
pub const NOMINAL_DT: f32 = {cfg_vals['nominal_dt']:.6f};

/// Retained alias for nominal DT (used for initial static filter cutoffs)
pub const DT: f32 = NOMINAL_DT;

/// LSM6DS3 register value for Accel (CTRL1_XL): configured ODR + +/-16g range
pub const IMU_ODR_XL_REG_VAL: u8 = {hex(cfg_vals['imu_xl_reg'])};

/// LSM6DS3 register value for Gyro (CTRL2_G): configured ODR + +/-2000 dps range
pub const IMU_ODR_G_REG_VAL: u8 = {hex(cfg_vals['imu_g_reg'])};

/// Number of raw IMU samples boxcar-averaged per flight loop iteration (1 = no decimation)
pub const IMU_DECIMATION: u8 = {cfg_vals['imu_decimation']};

/// Duration without valid RC packets before entering failsafe mode
pub const FAILSAFE_TIMEOUT: Duration = Duration::from_millis({cfg_vals['failsafe_ms']});


/// Configured RC channel indexes (0-indexed)
pub const RC_CH_ROLL: usize = {cfg_vals['ch_roll_idx']};
pub const RC_CH_PITCH: usize = {cfg_vals['ch_pitch_idx']};
pub const RC_CH_THROTTLE: usize = {cfg_vals['ch_throttle_idx']};
pub const RC_CH_YAW: usize = {cfg_vals['ch_yaw_idx']};
pub const RC_CH_ARM: usize = {cfg_vals['ch_arm_idx']};
pub const RC_CH_MODE: usize = {cfg_vals['ch_mode_idx']};

/// INAV-style multi-switch Flight Mode Activation Matrix
pub const MODE_MATRIX: ModeMatrix<{cfg_vals['mode_count']}> = {cfg_vals['mode_matrix_rust']};

/// Microsecond threshold for Arm Switch (CH5 / AUX1) above which aircraft arms
pub const ARM_SWITCH_THRESHOLD: u16 = {cfg_vals['arm_switch_threshold_us']};

/// Maximum allowable throttle pulse in microseconds to permit arming
pub const THROTTLE_ZERO_THRESHOLD: u16 = {cfg_vals['throttle_zero_threshold_us']};

/// Configured PWM frame frequency for servo outputs in Hz ({MIN_PWM_HZ}Hz - {MAX_PWM_HZ}Hz).
/// Standard analog servos require 50 Hz; digital servos can accept up to 400 Hz.
pub const SERVO_PWM_HZ: u32 = {cfg_vals['servo_pwm_hz']};

/// Per-servo output endpoint, subtrim, and inversion calibration profiles
pub const SERVO_CONFIGS: [ServoConfig; 8] = [
    {cfg_vals['servo1_rust']},
    {cfg_vals['servo2_rust']},
    {cfg_vals['servo3_rust']},
    {cfg_vals['servo4_rust']},
    {cfg_vals['servo5_rust']},
    {cfg_vals['servo6_rust']},
    {cfg_vals['servo7_rust']},
    {cfg_vals['servo8_rust']},
];

/// Configured PWM frame frequency for analog ESC output in Hz ({MIN_PWM_HZ}Hz - {MAX_PWM_HZ}Hz).
/// Legacy airplane ESCs require 50 Hz; high-speed multirotor ESCs support 400 Hz.
pub const ESC_PWM_HZ: u32 = {cfg_vals['motor_pwm_hz']};

/// Differential thrust authority for twin-motor airframes (0.0 .. 1.0 fraction of throttle range)
pub const DIFFERENTIAL_THRUST: f32 = {cfg_vals['diff_thrust']:.4f};

/// Maximum commanded roll rate in Acro / Rates mode (rad/s)
pub const MAX_ROLL_RATE: f32 = {cfg_vals['roll_dps']:.1f} * DEG_TO_RAD;

/// Maximum commanded pitch rate in Acro / Rates mode (rad/s)
pub const MAX_PITCH_RATE: f32 = {cfg_vals['pitch_dps']:.1f} * DEG_TO_RAD;

/// Maximum commanded yaw rate in Acro / Rates mode (rad/s)
pub const MAX_YAW_RATE: f32 = {cfg_vals['yaw_dps']:.1f} * DEG_TO_RAD;

/// Maximum bank angle in Angle / Self-Leveling mode (rad)
pub const MAX_ROLL_ANGLE: f32 = {cfg_vals['max_roll_deg']:.1f} * DEG_TO_RAD;

/// Maximum pitch angle in Angle / Self-Leveling mode (rad)
pub const MAX_PITCH_ANGLE: f32 = {cfg_vals['max_pitch_deg']:.1f} * DEG_TO_RAD;

/// Outer proportional gain converting attitude angle error to target rate in Angle mode (s^-1)
pub const ANGLE_P_GAIN: f32 = {cfg_vals['angle_p_gain']:.2f};

/// In-flight real-time PID potentiometer tuning configuration
pub const IN_FLIGHT_TUNING_ENABLED: bool = {cfg_vals['ift_enabled']};
pub const IN_FLIGHT_TUNING: InFlightTuning = InFlightTuning {{
    selector_channel_idx: {cfg_vals['ift_selector_idx']},
    num_banks: {cfg_vals['ift_num_banks']},
    banks: {cfg_vals['ift_banks_rust']},
}};

/// Emergency hard-deck floor in meters Above Ground Level (AGL)
pub const HARD_DECK_AGL_M: f32 = {cfg_vals['hard_deck_m']:.1f};

/// Barometer configuration
#[cfg(feature = "baro")]
pub use crate::drivers::BaroModel;

pub const BARO_ENABLED: bool = {str(cfg_vals['baro_enabled']).lower()};
#[cfg(feature = "baro")]
pub const BARO_MODEL: BaroModel = {cfg_vals['baro_model_rust']};
#[cfg(feature = "baro")]
pub const BARO_I2C_ADDR: u8 = {cfg_vals['baro_i2c_addr']};

/// External Compass / Magnetometer configuration
#[cfg(feature = "compass")]
pub use crate::drivers::CompassModel;

pub const COMPASS_ENABLED: bool = {str(cfg_vals['compass_enabled']).lower()};
#[cfg(feature = "compass")]
pub const COMPASS_MODEL: CompassModel = {cfg_vals['compass_model_rust']};
#[cfg(feature = "compass")]
pub const MAG_DECLINATION_RAD: f32 = {cfg_vals['compass_declination_rad']};
#[cfg(feature = "compass")]
pub const MAG_CALIBRATION: wingfc_core::compass::MagCalibration = wingfc_core::compass::MagCalibration {{
    hard_iron: {cfg_vals['compass_hard_iron_rust']},
    soft_iron: {cfg_vals['compass_soft_iron_rust']},
}};

/// GPS autonomous navigation and Return-to-Home parameters
pub const GPS_ENABLED: bool = {str(cfg_vals['gps_enabled']).lower()};
pub const RTH_MIN_SATS: u8 = {cfg_vals['rth_min_sats']};
pub const RTH_MAX_HDOP: f32 = {cfg_vals['rth_max_hdop']:.2f};
pub const RTH_SAFE_ALTITUDE_M: f32 = {cfg_vals['rth_safe_alt']:.1f};

/// Statutory operational ceiling limiter in meters Above Ground Level (AGL)
/// Note: FAA / EASA standard legal limit is 120.0 meters (400 ft) AGL.
pub const MAX_CEILING_AGL_M: f32 = {cfg_vals['max_ceiling_m']:.1f};

/// PT1 low-pass filter cutoff frequency for gyro harmonic vibration rejection (Hz)
pub const GYRO_LPF_CUTOFF_HZ: f32 = {cfg_vals['gyro_lpf']:.1f};

/// Madgwick AHRS filter convergence gain beta
pub const AHRS_BETA: f32 = {cfg_vals['ahrs_beta']:.4f};

/// Inner rate loop PID + FeedForward configuration
pub const RATE_PID_CONFIG: PidConfig = PidConfig {{
    kp: {cfg_vals['kp']:.4f},
    ki: {cfg_vals['ki']:.4f},
    kd: {cfg_vals['kd']:.4f},
    kff: {cfg_vals['kff']:.4f},
    max_integral: {cfg_vals['max_integral']:.4f},
    dterm_lpf_hz: {cfg_vals['dterm_lpf']:.1f},
    ff_lpf_hz: {cfg_vals['ff_lpf']:.1f},
}};
"""
    GENERATED_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(GENERATED_CONFIG_PATH, "w") as f:
        f.write(code)

    try:
        subprocess.run(["rustfmt", str(GENERATED_CONFIG_PATH)], capture_output=True, check=False)
    except Exception:
        pass


def print_advisories(advisories):
    """Prints highlighted warnings/notices if triggered."""
    if not advisories:
        return
    print("\n" + "!" * 76)
    print("⚠️   CONFIGURATION & HARDWARE ADVISORIES")
    print("!" * 76)
    for adv in advisories:
        print(f"\n{adv}")
    print("\n" + "!" * 76 + "\n")


def run_cmd(cmd, cwd=WORKSPACE_ROOT, capture=False):
    """Executes a subprocess command."""
    if capture:
        res = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return res.returncode == 0, res.stdout, res.stderr
    else:
        res = subprocess.run(cmd, cwd=cwd)
        return res.returncode == 0


def parse_llvm_size(elf_path):
    """Parses text, data, bss from llvm-size / size."""
    size_bin = shutil.which("llvm-size") or shutil.which("size")
    if not size_bin:
        return None, None, None

    ok, out, _ = run_cmd([size_bin, str(elf_path)], capture=True)
    if not ok:
        return None, None, None

    lines = out.strip().splitlines()
    if len(lines) >= 2:
        parts = lines[1].split()
        if len(parts) >= 3:
            try:
                return int(parts[0]), int(parts[1]), int(parts[2])
            except ValueError:
                pass
    return None, None, None


def cmd_check(cfg_vals):
    """Validates configuration and checks compilation of firmware."""
    print("=" * 70)
    print("WINGFC-RS CONFIGURATION CHECK")
    print("=" * 70)
    print(f"Target Board     : {cfg_vals['board']}")
    print(f"Build Profile    : {cfg_vals['profile'].upper()}")
    print(f"Receiver         : {cfg_vals['protocol'].upper()} (failsafe: {cfg_vals['failsafe_ms']}ms)")
    print(f"Arm Thresholds   : Arm Switch > {cfg_vals['arm_switch_threshold_us']} μs, Throttle <= {cfg_vals['throttle_zero_threshold_us']} μs")
    print(f"Servo Output     : {cfg_vals['servo_pwm_hz']} Hz PWM")
    print(f"Loop Timing      : {cfg_vals['nominal_loop_hz']} Hz ({cfg_vals['loop_rate_name']}) - Hardware DRDY Event Paced")
    if cfg_vals["motor_proto"] == "dshot":

        print(f"Motor Protocol   : DSHOT (Digital)")
    else:
        print(f"Motor Protocol   : PWM ({cfg_vals['motor_pwm_hz']} Hz)")
    print(f"Navigation       : {'ENABLED' if cfg_vals['nav_enabled'] else 'DISABLED'}")
    print(f"GPS Navigation   : {'ENABLED (Sense Plus UARTE1 @ 115.2k)' if cfg_vals['gps_enabled'] else 'DISABLED'}")
    print(f"Altitude Ceiling : {cfg_vals['max_ceiling_m']:.1f} m AGL (Hard deck: {cfg_vals['hard_deck_m']:.1f} m AGL)")
    print(f"Cargo Features   : --features {cfg_vals['features_str']}")
    print("-" * 70)

    print_advisories(cfg_vals["advisories"])

    generate_rust_config(cfg_vals)
    print("-> Checking compilation for target Cortex-M4F (thumbv7em-none-eabihf)... ", end="", flush=True)

    cmd = [
        "cargo", "check", "-p", "wingfc-firmware",
        "--target", TARGET,
        "--no-default-features",
        "--features", cfg_vals["features_str"],
    ]

    t0 = time.time()
    ok, _, stderr = run_cmd(cmd, capture=True)
    elapsed = time.time() - t0

    if ok:
        print(f"SUCCESS in {elapsed:.2f}s!")
        print("Your aircraft configuration is valid and ready to build.")
        return True
    else:
        print(f"FAILED ({elapsed:.2f}s)")
        print(stderr)
        return False


def cmd_build(cfg_vals, out_dir=DIST_DIR):
    """Builds release firmware and packages artifacts."""
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("WINGFC-RS FIRMWARE RELEASE BUILDER")
    print("=" * 70)
    print(f"Target Board     : {cfg_vals['board']}")
    print(f"Receiver         : {cfg_vals['protocol'].upper()} (failsafe: {cfg_vals['failsafe_ms']}ms)")
    print(f"Arm Thresholds   : Arm Switch > {cfg_vals['arm_switch_threshold_us']} μs, Throttle <= {cfg_vals['throttle_zero_threshold_us']} μs")
    print(f"Servo Output     : {cfg_vals['servo_pwm_hz']} Hz PWM")
    if cfg_vals["motor_proto"] == "dshot":
        print(f"Motor Protocol   : DSHOT (Digital)")
    else:
        print(f"Motor Protocol   : PWM ({cfg_vals['motor_pwm_hz']} Hz)")
    print(f"Navigation       : {'ENABLED' if cfg_vals['nav_enabled'] else 'DISABLED'}")
    print(f"GPS Navigation   : {'ENABLED (Sense Plus UARTE1 @ 115.2k)' if cfg_vals['gps_enabled'] else 'DISABLED'}")
    print(f"Altitude Ceiling : {cfg_vals['max_ceiling_m']:.1f} m AGL (Hard deck: {cfg_vals['hard_deck_m']:.1f} m AGL)")
    print(f"Cargo Features   : --features {cfg_vals['features_str']}")
    print(f"Output Directory : {out_dir}")
    print("-" * 70)

    print_advisories(cfg_vals["advisories"])

    generate_rust_config(cfg_vals)

    cmd = [
        "cargo", "build", "--release", "-p", "wingfc-firmware",
        "--target", TARGET,
        "--no-default-features",
        "--features", cfg_vals["features_str"],
    ]

    t0 = time.time()
    print("-> Compiling firmware with LTO and size optimization... ", end="", flush=True)
    ok, _, stderr = run_cmd(cmd, capture=True)
    elapsed = time.time() - t0

    built_elf = WORKSPACE_ROOT / "target" / TARGET / "release" / "wingfc-firmware"
    if not ok or not built_elf.exists():
        print(f"FAILED ({elapsed:.2f}s)")
        print(stderr)
        return False

    print(f"DONE ({elapsed:.1f}s)")

    # Output artifact paths
    base_name = f"wingfc-firmware-{cfg_vals['variant_name']}"
    dest_elf = out_dir / f"{base_name}.elf"
    dest_bin = out_dir / f"{base_name}.bin"
    dest_hex = out_dir / f"{base_name}.hex"

    shutil.copy2(built_elf, dest_elf)

    # Conversion tools
    objcopy_bin = shutil.which("llvm-objcopy") or shutil.which("rust-objcopy") or shutil.which("arm-none-eabi-objcopy")
    if objcopy_bin:
        run_cmd([objcopy_bin, "-O", "binary", str(dest_elf), str(dest_bin)], capture=True)
        run_cmd([objcopy_bin, "-O", "ihex", str(dest_elf), str(dest_hex)], capture=True)

    # Generate .uf2 artifact via uf2conv.py if available
    uf2conv_script = WORKSPACE_ROOT / "scripts" / "uf2conv" / "uf2conv.py"
    dest_uf2 = out_dir / f"{base_name}.uf2"
    if uf2conv_script.exists() and dest_hex.exists():
        run_cmd(["python3", str(uf2conv_script), "-c", "-f", "0xADA52840", str(dest_hex), "-o", str(dest_uf2)], capture=True)

    # Calculate memory consumption
    text, data, bss = parse_llvm_size(dest_elf)
    flash_used = (text + data) if text is not None else (dest_bin.stat().st_size if dest_bin.exists() else 0)
    ram_used = (data + bss) if data is not None else 0

    flash_pct = (flash_used / FLASH_TOTAL_BYTES) * 100.0
    ram_pct = (ram_used / RAM_TOTAL_BYTES) * 100.0

    # Calculate checksums
    hashes = {}
    for p in [dest_elf, dest_bin, dest_hex, dest_uf2]:
        if p.exists():
            h = hashlib.sha256(p.read_bytes()).hexdigest()
            hashes[p.name] = h

    sums_file = out_dir / "SHA256SUMS"
    with open(sums_file, "a") as f:
        for fname, h in sorted(hashes.items()):
            f.writelines(f"{h}  {fname}\n")

    print("\n" + "=" * 70)
    print("BUILD SUMMARY & RESOURCE CONSUMPTION")
    print("=" * 70)
    print(f"Firmware Binary  : {dest_elf.name}")
    print(f"Flash Memory     : {flash_used / 1024:.1f} KB / 1024 KB ({flash_pct:.1f}% used, {100 - flash_pct:.1f}% free)")
    print(f"RAM Memory       : {ram_used / 1024:.1f} KB / 256 KB ({ram_pct:.1f}% used, {100 - ram_pct:.1f}% free)")
    print("-" * 70)
    print("Produced Artifacts:")
    for fname, h in sorted(hashes.items()):
        print(f"  - {fname:<36} (SHA256: {h[:16]}...)")
    print("-" * 70)
    print("FLASHING INSTRUCTIONS:")
    print("  USB Drag-and-Drop (UF2 Bootloader):")
    print("    1. Double-tap reset button to mount XIAO-SENSE drive")
    print(f"    2. Copy '{dest_uf2.name}' to XIAO-SENSE drive")
    print(f"    (Manual conversion: python3 scripts/uf2conv/uf2conv.py -c -f 0xADA52840 {dest_hex.name} -o firmware.uf2)")
    print("  SWD / Debug Probe (probe-rs):")
    print(f"    probe-rs run --chip nRF52840_xxAA {dest_elf}")
    print("  Nordic nrfjprog:")
    print(f"    nrfjprog --program {dest_hex} --sectorerase --reset")
    print("=" * 70)
    return True


def cmd_init():
    """Restores default wingfc.yaml."""
    if DEFAULT_CONFIG_PATH.exists():
        backup_path = WORKSPACE_ROOT / "wingfc.yaml.bak"
        shutil.copy2(DEFAULT_CONFIG_PATH, backup_path)
        print(f"Existing wingfc.yaml backed up to {backup_path.name}")

    default_content = """# ==============================================================================
# WingFC-rs Aircraft Configuration File (wingfc.yaml)
# ==============================================================================
board: xiao-sense

receiver:
  protocol: crsf
  failsafe_timeout_ms: 500

servos:
  # PWM frame refresh frequency (50 Hz to 400 Hz).
  # WARNING: Standard analog servos MUST use 50 Hz. Exceeding 60 Hz burns them out!
  # High-speed digital servos can accept 200 Hz to 400 Hz.
  pwm_frequency_hz: 50

motor:
  protocol: pwm
  # PWM frequency (50 Hz to 400 Hz). Multirotor ESCs: 400 Hz. Airplane ESCs: 50 Hz.
  pwm_frequency_hz: 400
  min_throttle_us: 1000
  max_throttle_us: 2000

navigation:
  enabled: false
  barometer: false
  compass: false
  hard_deck_agl_m: 15.0
  # LEGALITY WARNING: Max allowable sUAS altitude is 120.0 m (400 ft) AGL under
  # FAA Part 107/44809 and EASA (EU) 2019/947 without formal waiver.
  max_ceiling_agl_m: 120.0

tuning:
  acro_rates:
    roll_dps: 600.0
    pitch_dps: 200.0
    yaw_dps: 100.0
  angle_limits:
    max_roll_deg: 60.0
    max_pitch_deg: 45.0
  filters:
    gyro_lpf_hz: 90.0
    dterm_lpf_hz: 40.0
    feedforward_lpf_hz: 45.0
  rate_pid:
    kp: 0.15
    ki: 0.05
    kd: 0.008
    kff: 0.04
    max_integral: 0.40
  ahrs_beta: 0.04

safety:
  max_arm_throttle_us: 1050
  finger_delay_ms: 200
  throttle_ramp_ms: 400
"""
    with open(DEFAULT_CONFIG_PATH, "w") as f:
        f.write(default_content)
    print(f"Created default configuration file: {DEFAULT_CONFIG_PATH}")


def find_uf2_mount():
    """Searches common mount locations for a UF2 bootloader drive."""
    user = os.environ.get("USER", "")
    search_dirs = [
        Path("/run/media") / user,
        Path("/media") / user,
        Path("/media"),
        Path("/mnt"),
        Path("/Volumes"),
    ]
    for parent in search_dirs:
        if parent.exists() and parent.is_dir():
            try:
                for child in parent.iterdir():
                    if child.is_dir():
                        if (child / "INFO_UF2.TXT").exists() or child.name.upper() in ["XIAO-SENSE", "XIAO-BLE", "NRF52BOOT"]:
                            return child
            except PermissionError:
                continue
    return None


def find_serial_port():
    """Finds first available serial port matching CDC ACM / USB for DFU bootloader."""
    candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/cu.usbmodem*"))
    return candidates[0] if candidates else None


def cmd_flash(cfg_vals, method=None, port=None, mount_path=None, out_dir=DIST_DIR):
    """Flashes firmware using selected method (auto, uf2, nrfjprog, dfu, probe-rs-hex, probe-rs-run)."""
    base_name = f"wingfc-firmware-{cfg_vals['variant_name']}"
    out_dir = Path(out_dir).resolve()
    elf_path = out_dir / f"{base_name}.elf"
    hex_path = out_dir / f"{base_name}.hex"
    bin_path = out_dir / f"{base_name}.bin"
    uf2_path = out_dir / f"{base_name}.uf2"

    if not elf_path.exists() or not hex_path.exists():
        print(f"Target artifacts for '{base_name}' not found. Building now...")
        ok = cmd_build(cfg_vals, out_dir)
        if not ok:
            return False

    selected_method = method or cfg_vals.get("flash_method", "auto")
    print("\n" + "=" * 70)
    print(f"WINGFC-RS FIRMWARE FLASH ENGINE: {base_name}")
    print("=" * 70)

    if selected_method == "auto":
        detected_mount = find_uf2_mount()
        nrfjprog_bin = shutil.which("nrfjprog")
        dfu_bin = shutil.which("adafruit-nrfutil")
        detected_port = find_serial_port()
        probe_bin = shutil.which("probe-rs")

        if detected_mount:
            selected_method = "uf2"
            mount_path = detected_mount
            print(f"-> Auto-detected mounted UF2 bootloader drive: {mount_path}")
        elif nrfjprog_bin:
            selected_method = "nrfjprog"
            print("-> Auto-detected Nordic nrfjprog programmer.")
        elif dfu_bin and detected_port:
            selected_method = "dfu"
            port = detected_port
            print(f"-> Auto-detected adafruit-nrfutil and serial device: {port}")
        elif probe_bin:
            selected_method = "probe-rs-hex"
            print("-> Auto-detected probe-rs debug tool.")
        else:
            print("No active hardware flashing tool or mounted drive detected automatically.")
            print("\nAvailable Flashing Methods & Manual Commands:")
            print("  1. USB Drag-and-Drop (UF2 Bootloader):")
            print("     - Double-tap the tiny reset button to mount 'XIAO-SENSE'.")
            print(f"     - Copy '{uf2_path.name}' into the drive:")
            print(f"       cp {uf2_path} /run/media/$USER/XIAO-SENSE/")
            print("  2. Nordic nrfjprog (SWD Intel HEX):")
            print(f"     nrfjprog --program {hex_path} --sectorerase --reset")
            print("  3. Serial DFU (adafruit-nrfutil):")
            print(f"     adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application {hex_path} dfu.zip")
            print("     adafruit-nrfutil dfu serial --package dfu.zip -p /dev/ttyACM0 -b 115200")
            print("  4. probe-rs (SWD):")
            print(f"     probe-rs download --chip nRF52840_xxAA --format hex {hex_path}")
            print(f"     probe-rs run --chip nRF52840_xxAA {elf_path}")
            return False

    if selected_method == "uf2":
        dest_mount = Path(mount_path) if mount_path else find_uf2_mount()
        if not dest_mount or not dest_mount.exists():
            print("Searching for UF2 bootloader drive ('XIAO-SENSE')...")
            print("  --> Please double-tap the tiny reset button on your XIAO nRF52840 Sense now.")
            for sec in range(15, 0, -1):
                sys.stdout.write(f"\r  Waiting for drive to mount... ({sec}s remaining) ")
                sys.stdout.flush()
                time.sleep(1)
                dest_mount = find_uf2_mount()
                if dest_mount and dest_mount.exists():
                    sys.stdout.write("\r" + " " * 55 + "\r")
                    print(f"-> Detected mounted UF2 drive: {dest_mount}")
                    break

        if not dest_mount or not dest_mount.exists():
            print("\nERROR: UF2 bootloader drive not detected.")
            print("  Make sure your Seeed XIAO is connected via USB-C and double-tap the reset button quickly.")
            print("  You can also specify the path manually with: ./scripts/build.sh flash --method uf2 --mount /path/to/XIAO-SENSE")
            return False

        if not uf2_path.exists():
            uf2conv_script = WORKSPACE_ROOT / "scripts" / "uf2conv" / "uf2conv.py"
            if uf2conv_script.exists() and hex_path.exists():
                run_cmd(["python3", str(uf2conv_script), "-c", "-f", "0xADA52840", str(hex_path), "-o", str(uf2_path)], capture=True)

        target_file = dest_mount / uf2_path.name
        print(f"Copying '{uf2_path.name}' -> '{target_file}'...")
        try:
            with open(uf2_path, "rb") as fsrc:
                with open(target_file, "wb") as fdst:
                    shutil.copyfileobj(fsrc, fdst, 64 * 1024)
            print("Flashing successful! XIAO bootloader will automatically reboot and start WingFC.")
            return True
        except (OSError, IOError) as e:
            # When the last block of the UF2 payload is written, the UF2 bootloader
            # immediately resets the MCU. This causes the virtual FAT USB drive to
            # disconnect before the OS completes file close/stat operations.
            time.sleep(0.5)
            if not dest_mount.exists():
                print("Flashing successful! XIAO bootloader received firmware and rebooted.")
                return True
            print(f"ERROR copying UF2 file: {e}")
            return False

    elif selected_method == "nrfjprog":
        if not shutil.which("nrfjprog"):
            print("ERROR: 'nrfjprog' not found in system PATH. Install Nordic Command Line Tools.")
            return False
        cmd = ["nrfjprog", "--program", str(hex_path), "--sectorerase", "--reset"]
        print(f"Executing: {' '.join(cmd)}")
        return run_cmd(cmd)

    elif selected_method == "dfu":
        dfu_tool = shutil.which("adafruit-nrfutil")
        if not dfu_tool:
            print("ERROR: 'adafruit-nrfutil' not found in system PATH. Install via pip install adafruit-nrfutil.")
            return False
        target_port = port or (cfg_vals.get("flash_port") if cfg_vals.get("flash_port") != "auto" else None) or find_serial_port()
        if not target_port:
            print("ERROR: No serial port found. Connect your board in bootloader mode or specify --port <device>.")
            return False

        zip_pkg = out_dir / f"{base_name}_dfu.zip"
        gen_cmd = [dfu_tool, "dfu", "genpkg", "--dev-type", "0x0052", "--application", str(hex_path), str(zip_pkg)]
        print(f"Generating DFU package: {' '.join(gen_cmd)}")
        if not run_cmd(gen_cmd):
            return False

        flash_cmd = [dfu_tool, "dfu", "serial", "--package", str(zip_pkg), "-p", str(target_port), "-b", "115200"]
        print(f"Flashing DFU package: {' '.join(flash_cmd)}")
        return run_cmd(flash_cmd)

    elif selected_method in ["probe-rs", "probe-rs-hex"]:
        if not shutil.which("probe-rs"):
            print("ERROR: 'probe-rs' not found in system PATH. Install via cargo install probe-rs-tools.")
            return False
        cmd = ["probe-rs", "download", "--chip", "nRF52840_xxAA", "--format", "hex", str(hex_path)]
        print(f"Executing: {' '.join(cmd)}")
        return run_cmd(cmd)

    elif selected_method == "probe-rs-run":
        if not shutil.which("probe-rs"):
            print("ERROR: 'probe-rs' not found in system PATH. Install via cargo install probe-rs-tools.")
            return False
        cmd = ["probe-rs", "run", "--chip", "nRF52840_xxAA", str(elf_path)]
        print(f"Executing: {' '.join(cmd)}")
        return run_cmd(cmd)

    else:
        print(f"ERROR: Unknown flashing method '{selected_method}'.")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="WingFC-rs User-Friendly Build & Aircraft Configuration Utility"
    )
    parser.add_argument(
        "action",
        nargs="?",
        default="build",
        choices=["build", "check", "test", "matrix", "init", "flash", "release"],
        help="Action: 'build' (default: build from wingfc.yaml), 'check' (validate & check syntax), 'test' (run host unit tests), 'matrix' (run 8-target release matrix), 'init' (create fresh wingfc.yaml), 'flash' (flash via probe-rs)",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to aircraft YAML configuration file (default: wingfc.yaml)",
    )
    parser.add_argument(
        "--out-dir",
        default=str(DIST_DIR),
        help="Output directory for built release binaries (default: dist/)",
    )
    parser.add_argument(
        "--method",
        "-m",
        default=None,
        choices=["auto", "nrfjprog", "uf2", "dfu", "probe-rs-hex", "probe-rs", "probe-rs-run"],
        help="Flashing method: 'nrfjprog' (SWD Intel HEX), 'uf2' (USB mass storage drag-and-drop), 'dfu' (adafruit-nrfutil serial), 'probe-rs-hex' (probe-rs HEX download), 'probe-rs-run' (probe-rs ELF with RTT), 'auto' (detect connected device)",
    )
    parser.add_argument(
        "--port",
        "-p",
        default=None,
        help="Serial port for DFU flashing (e.g., /dev/ttyACM0). Auto-detected if not specified.",
    )
    parser.add_argument(
        "--mount",
        default=None,
        help="Mount directory for UF2 drive (e.g., /run/media/user/XIAO-SENSE). Auto-detected if not specified.",
    )

    args, extra = parser.parse_known_args()

    if args.action == "init":
        cmd_init()
        return

    if args.action == "test":
        print("Running host unit tests (wingfc-core)...")
        cmd = ["cargo", "test", "-p", "wingfc-core", "--all-features"]
        ok = run_cmd(cmd)
        sys.exit(0 if ok else 1)

    if args.action == "matrix":
        matrix_script = WORKSPACE_ROOT / "scripts" / "build_matrix.py"
        cmd = ["python3", str(matrix_script)] + extra
        ok = run_cmd(cmd)
        sys.exit(0 if ok else 1)

    if args.action == "release":
        release_script = WORKSPACE_ROOT / "scripts" / "release.sh"
        cmd = ["bash", str(release_script)] + extra
        ok = run_cmd(cmd)
        sys.exit(0 if ok else 1)

    config_path = Path(args.config).resolve()
    raw_cfg = load_config(config_path)
    cfg_vals = validate_config(raw_cfg)

    if args.action == "check":
        ok = cmd_check(cfg_vals)
        sys.exit(0 if ok else 1)
    elif args.action == "build":
        ok = cmd_build(cfg_vals, args.out_dir)
        sys.exit(0 if ok else 1)
    elif args.action == "flash":
        ok = cmd_flash(
            cfg_vals,
            method=args.method,
            port=args.port,
            mount_path=args.mount,
            out_dir=args.out_dir,
        )
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
