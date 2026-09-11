# WingFC-rs

High-performance, zero-latency, bare-metal Rust flight controller firmware for sub-250g FPV flying wings, acrobatic wings, and autonomous UAVs.

## Key Features
- **YAML Aircraft Configuration:** Configure board, receiver, motor, servos, safety thresholds, navigation, rates, and PID tunes in [`wingfc.yaml`](wingfc.yaml) without touching build files or Rust code.
- **Deterministic Hard Real-Time Execution:** Zero garbage collector, zero heap allocation (`#![no_std]`), and 500 Hz control loop (2000 &mu;s loop time).
- **Acrobatic & Rates Mode:** Direct angular velocity tracking with PT1 vibration filtering and FeedForward.
- **Madgwick Quaternion AHRS:** 6-DoF and 9-DoF attitude estimation without gimbal lock and with dynamic G-rejection.
- **Autonomous Navigation Suite:** Barometric Vertical Kalman Filter (altitude hold, stall prevention, hard-deck floor, statutory 120m ceiling limiter), tilt-compensated magnetic compass, and GPS Return-to-Home (RTH).
- **Zero-Allocation NMEA GPS Engine:** High-speed streaming parser with automatic launch point latching and equirectangular flat-Earth coordinate projection.
- **DMA-Driven Hardware I/O:** Embassy-based async UART (CRSF / ELRS @ 420 kBaud, iBus @ 115.2 kBaud, GPS @ 115.2 kBaud), SPI (DShot), and multi-frequency PWM (50 Hz &ndash; 400 Hz).
- **Bidirectional CRSF / ELRS Telemetry:** Broadcasts 3D vehicle attitude, active flight mode, battery status, and GPS coordinates to EdgeTX / OpenTX transmitters via EasyDMA UARTE0 TX (`D6` / `P1.11`).
- **Configurable Arming Interlocks:** Pilot-configurable arm switch and throttle-zero thresholds with active motor spin-up warning systems.
- **Next-Gen Telemetry & Bench Debugging:** Sub-microsecond RAM logging over SEGGER RTT via SWD, `defmt` structured logs, and 500 Hz binary blackbox structures.

## Documentation
- ⚙️ [Aircraft Configuration Guide (`wingfc.yaml`)](docs/CONFIG.md)
- 📖 [System Architecture & Control Loop Specification](docs/ARCHITECTURE.md)
- 📊 [System Architecture & Control Flowcharts](docs/FLOWCHARTS.md)
- 🔌 [Pin Configuration & Hardware Allocation Specification](docs/PINOUT.md)
- 🛰️ [GPS Navigation & Return-to-Home Specification](docs/GPS_NAVIGATION.md)
- 📊 [SEGGER RTT & Blackbox Logging Architecture](docs/RTT_BLACKBOX_LOGGING.md)
- 📡 [Receiver & Telemetry Specification (CRSF/ELRS & iBus)](docs/RECEIVERS.md)
- 🚀 [Production Release Matrix & Sizing](docs/RELEASE_MATRIX.md)
- ⚡ [Firmware Deployment & Flashing Guide](docs/FLASHING.md)
- 🤝 [Contributing Guidelines & Architecture Standards](CONTRIBUTING.md)
- 📝 [Changelog & Version History](CHANGELOG.md)

## User Quick Start (Zero Rust Knowledge Required)

Just like Betaflight, INAV, or EdgeTX, all aircraft settings are controlled via a single human-readable configuration file: [`wingfc.yaml`](wingfc.yaml).

```bash
# 1. Edit your aircraft settings in wingfc.yaml (board, receiver, motor, safety, rates, PIDs)
nano wingfc.yaml

# 2. Check configuration syntax and validity (< 0.5s)
./scripts/build.sh check

# 3. Build optimized release firmware (.elf, .bin, .hex in dist/)
./scripts/build.sh

# 4. Flash to Seeed Studio XIAO (see docs/FLASHING.md):
# Option A: Auto-detect connected programmer or mounted drive (Recommended)
./scripts/build.sh flash

# Option B: USB Drag-and-Drop (UF2 Bootloader - No probe needed)
# Double-tap reset button to mount XIAO-SENSE drive, then run:
./scripts/build.sh flash --method uf2
# (Or manual copy: cp dist/wingfc-firmware-*.uf2 /run/media/$USER/XIAO-SENSE/)

# Option C: Nordic J-Link SWD via self-addressing Intel HEX
./scripts/build.sh flash --method nrfjprog

# Option D: Direct SWD Debug Probe (probe-rs)
./scripts/build.sh flash --method probe-rs-run
```

---

## Dual-LED Visual Telemetry Matrix

WingFC-rs uses a synchronized dual-LED system: the primary **RGB LED** reports vehicle safety/arming modes, while the discrete **User LED (`P0.17`)** provides link quality and GPS lock feedback.

| LED Indicator | Pattern / Color | Meaning |
| :--- | :--- | :--- |
| **RGB LED** | **White (5 Hz)** | Stationary gyro calibration at boot. |
| **RGB LED** | **Yellow (5 Hz)** | ESC throttle calibration pass-through active. |
| **RGB LED** | **Yellow (Double Blink)** | Arming refused / safety lockout (throttle raised or switch armed at boot). |
| **RGB LED** | **Red (Slow 2 Hz)** | Failsafe / Link lost. |
| **RGB LED** | **Green (Solid)** | Disarmed & safe. Ready to arm. |
| **RGB LED** | **Blue (Solid)** | Armed (Acro / Rate mode). |
| **RGB LED** | **Cyan (Solid)** | Armed (Angle / Self-leveling mode). |
| **User LED (`P0.17`)** | **Heartbeat / Pulse** | *Non-GPS builds:* Solid ON when RC link is healthy; pulses on packet arrival.<br>*GPS builds:* 1 Hz pulse searching, 2 Hz double-flash on 3D fix, Solid ON when locked (&ge;8 sats). |

For complete cadence details and wiring specifications, see the [Pin Configuration Specification](docs/PINOUT.md).

---

## Workspace Structure
- `wingfc.yaml`: Master user aircraft configuration file.
- `crates/wingfc-core`: Pure `#![no_std]` algorithms (AHRS, PID, Airframe Mixers, Protocols, GPS) testable directly on host PC with `cargo test`.
- `crates/wingfc-firmware`: Target firmware for Seeed Studio XIAO nRF52840 Sense (`thumbv7em-none-eabihf`).
- `scripts/`: Build automation, validation, and release matrix scripts.
- `docs/`: Comprehensive technical, hardware, pinout, and configuration specifications.

## Advanced Building & Developer Testing

### Run Core Unit Tests on Host:
```bash
# Via build utility
./scripts/build.sh test

# Or directly via cargo
cargo test -p wingfc-core --all-features

# Generate detailed code coverage report via cargo-llvm-cov
cargo llvm-cov -p wingfc-core --all-features
```

#### Core Algorithm Test Coverage (88.4% Line, 89.6% Region, 87.8% Function)
The `#![no_std]` algorithmic flight core (`wingfc-core`) is verified by 27 host unit tests covering safety, kinematics, protocol decoders, and navigation:

| Subsystem / Module | Source File | Line Coverage | Function Coverage | Region Coverage | Key Verification Scenarios |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **Altitude Safety & Terrain** | [`altitude_safety.rs`](crates/wingfc-core/src/altitude_safety.rs) | **100.0%** | **100.0%** | **100.0%** | Ceiling override, floor intervention, pull-up ramp |
| **Airframe Mixer** | [`mixer.rs`](crates/wingfc-core/src/mixer.rs) | **100.0%** | **100.0%** | **100.0%** | Elevon, diff thrust, standard fixed wing, V-tail, trimming |
| **Protocol Types & Converters** | [`protocol/mod.rs`](crates/wingfc-core/src/protocol/mod.rs) | **100.0%** | **100.0%** | **100.0%** | Protocol conversions, channel normalization |
| **Hand Autolaunch** | [`autolaunch.rs`](crates/wingfc-core/src/autolaunch.rs) | **97.8%** | **100.0%** | **96.9%** | Throw acceleration trigger, motor delay, climb, stick abort |
| **Vertical Kalman Filter** | [`vertical_kf.rs`](crates/wingfc-core/src/vertical_kf.rs) | **97.9%** | **100.0%** | **98.1%** | Accel/baro sensor fusion, vertical velocity, state reset |
| **Signal Filtering** | [`filter.rs`](crates/wingfc-core/src/filter.rs) | **96.9%** | **100.0%** | **97.1%** | PT1 1st-order step response, 2nd-order Biquad low-pass |
| **CRSF Protocol & Telemetry** | [`protocol/crsf.rs`](crates/wingfc-core/src/protocol/crsf.rs) | **95.7%** | **91.7%** | **96.3%** | CRC8 DVB-S2, RC channels, telemetry frames (att/bat/gps/baro) |
| **Math & Quaternion Library** | [`math.rs`](crates/wingfc-core/src/math.rs) | **94.6%** | **95.0%** | **96.4%** | Vector cross/dot/norm, Quaternion multiplication/conjugate |
| **Arming Safety Interlocks** | [`safety.rs`](crates/wingfc-core/src/safety.rs) | **86.5%** | **77.8%** | **84.1%** | Configurable thresholds, high-throttle lockout, zero-surge |
| **Rate PID & FeedForward** | [`pid.rs`](crates/wingfc-core/src/pid.rs) | **84.5%** | **80.0%** | **82.8%** | Anti-windup clamping, dynamic FeedForward transient & decay |
| **Mode Matrix & Cascaded** | [`mode.rs`](crates/wingfc-core/src/mode.rs) | **82.9%** | **68.4%** | **81.8%** | Multi-switch mode ranges, Heading Hold & Alt Hold cascades |
| **In-Flight Pot Tuning** | [`tuning.rs`](crates/wingfc-core/src/tuning.rs) | **82.0%** | **100.0%** | **80.9%** | Potentiometer channel deadbands, real-time PID scaling |
| **FlySky iBus Protocol** | [`protocol/ibus.rs`](crates/wingfc-core/src/protocol/ibus.rs) | **82.3%** | **71.4%** | **85.7%** | 16-bit inverse checksum, 14-channel unpacking, failsafe bits |
| **GPS & NMEA Navigation** | [`gps.rs`](crates/wingfc-core/src/gps.rs) | **79.6%** | **85.7%** | **84.8%** | NMEA GGA/RMC parsing, home latching, RTH bearing & error |
| **Madgwick 9-DoF AHRS** | [`ahrs.rs`](crates/wingfc-core/src/ahrs.rs) | **79.5%** | **100.0%** | **84.3%** | 6-DoF gravity alignment, 9-DoF mag fusion, zero-mag fallback |
| **DShot Motor Protocol** | [`dshot.rs`](crates/wingfc-core/src/dshot.rs) | **76.9%** | **66.7%** | **82.9%** | 16-bit packet encoding, nibble checksum, SPI MOSI bitstream |
| **Total `wingfc-core`** | **All Modules** | **88.4%** | **87.8%** | **89.6%** | **28 unit tests, 100% passing** |

### Production Release Matrix (All 8 Release Permutations):
```bash
# Check all 8 permutations and host tests
./scripts/build.sh matrix check

# Build and package all 8 release binaries to dist/
./scripts/build.sh matrix build
```

| Variant | Receiver | Motor | Navigation (`nav`: Baro + Compass + GPS) |
| :--- | :--- | :--- | :--- |
| `wingfc-firmware-crsf-pwm` | CRSF | PWM 50&ndash;400Hz | OFF |
| `wingfc-firmware-crsf-pwm-nav` | CRSF | PWM 50&ndash;400Hz | ON |
| `wingfc-firmware-crsf-dshot` | CRSF | DShot | OFF |
| `wingfc-firmware-crsf-dshot-nav` | CRSF | DShot | ON |
| `wingfc-firmware-ibus-pwm` | iBus | PWM 50&ndash;400Hz | OFF |
| `wingfc-firmware-ibus-pwm-nav` | iBus | PWM 50&ndash;400Hz | ON |
| `wingfc-firmware-ibus-dshot` | iBus | DShot | OFF |
| `wingfc-firmware-ibus-dshot-nav` | iBus | DShot | ON |
