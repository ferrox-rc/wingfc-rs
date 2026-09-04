# WingFC-rs

High-performance, zero-latency, bare-metal Rust flight controller firmware for sub-250g FPV flying wings, acrobatic wings, and autonomous UAVs.

## Key Features
- **YAML Aircraft Configuration:** Configure board, receiver, motor, servos, safety thresholds, navigation, rates, and PID tunes in [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml) without touching build files or Rust code.
- **Deterministic Hard Real-Time Execution:** Zero garbage collector, zero heap allocation (`#![no_std]`), and 500 Hz control loop (2000 &mu;s loop time).
- **Acrobatic & Rates Mode:** Direct angular velocity tracking with PT1 vibration filtering and FeedForward.
- **Madgwick Quaternion AHRS:** 6-DoF and 9-DoF attitude estimation without gimbal lock and with dynamic G-rejection.
- **Autonomous Navigation Suite:** Barometric Vertical Kalman Filter (altitude hold, stall prevention, hard-deck floor, statutory 120m ceiling limiter), tilt-compensated magnetic compass, and GPS Return-to-Home (RTH).
- **Zero-Allocation NMEA GPS Engine:** High-speed streaming parser with automatic launch point latching and equirectangular flat-Earth coordinate projection.
- **DMA-Driven Hardware I/O:** Embassy-based async UART (CRSF / ELRS @ 420 kBaud, iBus @ 115.2 kBaud, GPS @ 115.2 kBaud), SPI (DShot), and multi-frequency PWM (50 Hz &ndash; 400 Hz).
- **Configurable Arming Interlocks:** Pilot-configurable arm switch and throttle-zero thresholds with active motor spin-up warning systems.
- **Next-Gen Telemetry & Bench Debugging:** Sub-microsecond RAM logging over SEGGER RTT via SWD, `defmt` structured logs, and 500 Hz binary blackbox structures.

## Documentation
- ⚙️ [Aircraft Configuration Guide (`wingfc.yaml`)](docs/CONFIG.md)
- 📖 [System Architecture & Control Loop Specification](docs/ARCHITECTURE.md)
- 🔌 [Pin Configuration & Hardware Allocation Specification](docs/PINOUT.md)
- 🛰️ [GPS Navigation & Return-to-Home Specification](docs/GPS_NAVIGATION.md)
- 📊 [SEGGER RTT & Blackbox Logging Architecture](docs/RTT_BLACKBOX_LOGGING.md)
- 🚀 [Production Release Matrix & Sizing](docs/RELEASE_MATRIX.md)

## User Quick Start (Zero Rust Knowledge Required)

Just like Betaflight, INAV, or EdgeTX, all aircraft settings are controlled via a single human-readable configuration file: [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml).

```bash
# 1. Edit your aircraft settings in wingfc.yaml (board, receiver, motor, safety, rates, PIDs)
nano wingfc.yaml

# 2. Check configuration syntax and validity (< 0.5s)
./scripts/build.sh check

# 3. Build optimized release firmware (.elf, .bin, .hex in dist/)
./scripts/build.sh

# 4. (Optional) Flash directly to Seeed Studio XIAO via SWD debug probe
./scripts/build.sh flash
```

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
```

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
