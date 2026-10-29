# wingfc-rs

<p align="center">
  <strong>Deterministic, Bare-Metal Rust Flight Controller for Sub-250g Autonomous UAVs & Flying Wings</strong><br>
  <em>The airborne flight companion in the <a href="https://ferroxrc.com">Ferrox-RC</a> avionics ecosystem</em>
</p>

<p align="center">
  <a href="https://github.com/ferrox-rc/wingfc-rs/releases"><img src="https://img.shields.io/github/v/release/ferrox-rc/wingfc-rs?style=flat-square&color=00E5FF" alt="Latest Release" /></a>
  <a href="https://github.com/ferrox-rc/wingfc-rs/actions"><img src="https://img.shields.io/badge/CI%2FCD-passing-brightgreen?style=flat-square" alt="CI/CD Status" /></a>
  <a href="https://ferroxrc.com"><img src="https://img.shields.io/badge/Ecosystem-ferroxrc.com-E05A47.svg?style=flat-square" alt="Website" /></a>
  <a href="https://github.com/ferrox-rc/flysky-i6x-rs"><img src="https://img.shields.io/badge/Companion%20OS-flysky--i6x--rs-00E676.svg?style=flat-square" alt="Ground Companion" /></a>
  <a href="https://github.com/ferrox-rc/wingfc-hardware"><img src="https://img.shields.io/badge/Hardware-wingfc--hardware-blueviolet.svg?style=flat-square" alt="Carrier Hardware" /></a>
  <img src="https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square" alt="License" />
</p>

---

## Overview

`wingfc-rs` delivers deterministic, hard real-time `#![no_std]` Rust execution for micro fixed-wing aircraft, sub-250g FPV flying wings, and autonomous UAVs.

Operating on the **Seeed Studio XIAO nRF52840** (ARM Cortex-M4F @ 64 MHz with single-cycle hardware FPU) and the cooperative **Embassy** async runtime, `wingfc-rs` replaces monolithic multirotor flight stacks with an architecture optimized specifically for fixed-wing aerodynamics.

```
┌────────────────────────────────┐       AFHDS 2A / CRSF / ELRS       ┌────────────────────────────────┐
│   Ground Transmitter Station   │ ◄────────────────────────────────► │     Airborne Receiver          │
│   (flysky-i6x-rs / EdgeTX)     │        Bidirectional RF            │     (CRSF / ELRS / iBus)       │
└────────────────────────────────┘                                    └───────────────┬────────────────┘
                                                                                      │ UARTE0 (420k / 115.2k)
                                                                                      ▼
                                                                      ┌────────────────────────────────┐
                                                                      │           wingfc-rs            │
                                                                      │   Seeed XIAO nRF52840 (64MHz)  │
                                                                      │   • 3-Tier Multi-Rate Loop     │
                                                                      │   • 1666 Hz IMU & Decimation   │
                                                                      │   • 208.33 Hz Flight Core      │
                                                                      │   • VQF AHRS Attitude Engine   │
                                                                      │   • 50 Hz Nav & QSPI Blackbox  │
                                                                      └───────────────┬────────────────┘
                                                                                      │ PWM0 / PWM1
                                                                                      ▼
                                                                      ┌────────────────────────────────┐
                                                                      │     Actuators & Propulsion     │
                                                                      │   • Left & Right Elevon Servos │
                                                                      │   • Brushless Motor ESC        │
                                                                      └────────────────────────────────┘
```

---

## Key Features

* **3-Tier Multi-Rate Architecture (`loop_rate: multirate`):**
  * **Tier 1 (1.666 kHz IMU Acquisition):** Hardware DRDY interrupt-driven sampling on dedicated `TWISPI0` with an 8:1 boxcar downsampler (`BoxcarAccumulator8`) providing sinc low-pass rejection of motor harmonics.
  * **Tier 2 (208.33 Hz Flight Control Loop):** Event-paced 4.8 ms control step running VQF AHRS attitude estimation, angular rate PIDs, and elevon mixing, updating autonomous hardware `PWM0/1/2` EasyDMA registers directly.
  * **Tier 3 (50 Hz Background Sensors & Telemetry):** External I2C navigation sensors (`TWISPI1`), CRSF telemetry broadcast, and onboard QSPI blackbox logging.
* **VQF Quaternion AHRS:** Versatile Quaternion-based Filter (Laidig & Seel) with strict decoupling of inclination (roll/pitch) from heading, inertial acceleration low-pass filtering, online gyro bias estimation, and magnetic disturbance rejection.
* **Native QSPI NOR Flash Blackbox:** Autonomous recording to the onboard 2 MB Puya P25Q16H flash at 50 Hz. Stores 32-byte binary frames with live roll/pitch PID terms (`[P, I, D]`), gyro rates, accelerations, actuator pulses, and CRC-8 integrity checks. Accompanied by `scripts/blackbox.py` for simultaneous `.bbl` (Betaflight Blackbox Explorer) and `.csv` export via USB or DebugMate SWD.
* **YAML Aircraft Configuration & Build Profiles:** Master configuration in [`wingfc.yaml`](wingfc.yaml) defining airframe geometry, receiver protocols, pin mappings, safety limits, and PID gains. Supports `ultralight` (sub-250g default) and `gps` (full navigation) high-level build profiles with granular compile-time feature flags.
* **Autonomous Navigation Suite:** Barometric Vertical Kalman Filter (altitude hold, stall prevention, statutory 120m ceiling limiter, ground-proximity floor), tilt-compensated magnetic compass calibration, and GPS Return-to-Home (RTH).
* **Sensor Health State Machine:** Tri-state monitoring (`Healthy`, `Degraded`, `Failed`) with timeout fault detection, consecutive sample recovery, and safe fallback logic (for example, falling back from `HeadingHold` or `AltHold` to `Angle` mode if a sensor degrades).
* **Zero-Allocation Streaming GPS Parser:** High-speed NMEA engine parsing GGA and RMC sentences on `UARTE1` with automatic launch point latching and equirectangular coordinate math.
* **Bench Diagnostics & RTT Telemetry:** Real-time structured logging over SEGGER RTT via SWD, supported out-of-the-box by the Seeed Studio XIAO DebugMate cradle.

---

## Documentation

* ⚙️ [Aircraft Configuration Guide (`wingfc.yaml`)](docs/CONFIG.md)
* 📖 [System Architecture & Multi-Rate Loop Specification](docs/ARCHITECTURE.md)
* 📊 [System Architecture & Control Flowcharts](docs/FLOWCHARTS.md)
* 📼 [Onboard QSPI Blackbox Flight Data Recorder](docs/BLACKBOX.md)
* 📊 [SEGGER RTT Telemetry & Bench Debugging Architecture](docs/RTT_BLACKBOX_LOGGING.md)
* 🔌 [Pin Configuration & Hardware Allocation Specification](docs/PINOUT.md)
* 🛠️ [Hardware Carrier Board Repository (KiCad Schematics & PCB Layout)](https://github.com/ferrox-rc/wingfc-hardware)
* 🛰️ [GPS Navigation & Return-to-Home Specification](docs/GPS_NAVIGATION.md)
* 📡 [Receiver & Telemetry Specification (CRSF/ELRS & iBus)](docs/RECEIVERS.md)
* 🚀 [Production Release Matrix & Sizing](docs/RELEASE_MATRIX.md)
* ⚡ [Firmware Deployment & Flashing Guide](docs/FLASHING.md)
* 🤝 [Contributing Guidelines & Architecture Standards](CONTRIBUTING.md)
* 📝 [Changelog & Version History](CHANGELOG.md)

---

## Quick Start (Zero Rust Knowledge Required)

Just like Betaflight, INAV, or EdgeTX, all aircraft settings are controlled through a single configuration file: [`wingfc.yaml`](wingfc.yaml).

```bash
# 1. Edit aircraft settings in wingfc.yaml (board, receiver, motor, safety, rates, PIDs)
nano wingfc.yaml

# 2. Check configuration syntax and validity (< 0.5s)
./scripts/build.sh check               # Windows: scripts\build.bat check

# 3. Build optimized release binaries (.uf2, .hex, .bin, .elf in dist/)
./scripts/build.sh                     # Windows: scripts\build.bat

# 4. Flash to Seeed Studio XIAO (see docs/FLASHING.md):
# Option A: Auto-detect connected programmer or mounted drive (Recommended)
./scripts/build.sh flash               # Windows: scripts\build.bat flash

# Option B: USB Drag-and-Drop (UF2 Bootloader)
# Double-tap the reset button to mount XIAO-SENSE, then run:
./scripts/build.sh flash --method uf2

# Option C: Direct SWD Debug Probe (Seeed Studio XIAO DebugMate or probe-rs)
./scripts/build.sh flash --method probe-rs-run
```

---

## Dual-LED Visual Telemetry Matrix

WingFC-rs provides vehicle feedback via the onboard **RGB LED** (safety, flight mode, arming state) and the discrete **User LED (`P0.17`)** (link status and GPS lock):

| LED Indicator | Pattern / Color | Vehicle State |
| :--- | :--- | :--- |
| **RGB LED** | **White (5 Hz)** | Stationary gyro calibration at boot. |
| **RGB LED** | **Yellow (5 Hz)** | ESC throttle calibration pass-through active. |
| **RGB LED** | **Yellow (Double Blink)** | Arming refused / safety lockout (throttle raised or switch armed at boot). |
| **RGB LED** | **Red (Slow 2 Hz)** | Failsafe / Link lost. |
| **RGB LED** | **Green (Solid)** | Disarmed and safe. Ready to arm. |
| **RGB LED** | **Blue (Solid)** | Armed (Acro / Rate mode). |
| **RGB LED** | **Cyan (Solid)** | Armed (Angle / Self-leveling mode). |
| **User LED (`P0.17`)** | **Heartbeat / Pulse** | *Non-GPS builds:* Solid ON when RC link is healthy; pulses on packet arrival.<br>*GPS builds:* 1 Hz pulse searching, 2 Hz double-flash on 3D fix, Solid ON when locked (&ge;8 sats). |

For complete wiring specifications and pad allocations, see the [Pin Configuration Specification](docs/PINOUT.md).

---

## Workspace Structure

* [`wingfc.yaml`](wingfc.yaml): Master user aircraft configuration file.
* [`crates/wingfc-core`](crates/wingfc-core): Pure `#![no_std]` algorithms (VQF AHRS, PID, downsampling, airframe mixers, protocols, GPS) testable directly on host platforms via `cargo test`.
* [`crates/wingfc-firmware`](crates/wingfc-firmware): Target firmware for Seeed Studio XIAO nRF52840 Sense (`thumbv7em-none-eabihf`).
* [`scripts/`](scripts): Build automation, validation, and blackbox offload tools.
* [`docs/`](docs): Technical specifications, hardware pinouts, and subsystem guides.

---

## Automated Verification & Test Coverage

The `#![no_std]` algorithmic flight core (`wingfc-core`) is verified by 31 host unit tests achieving **88.6% Line Coverage** and **89.9% Region Coverage**:

| Module | Source File | Line Cov | Branch Cov | Region Cov | Core Functionality Verified |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **CRSF Protocol & Telemetry** | [`protocol/crsf.rs`](crates/wingfc-core/src/protocol/crsf.rs) | **95.7%** | **91.7%** | **96.3%** | CRC8 DVB-S2, channel pack/unpack, telemetry frames |
| **Math & Quaternion Library** | [`math.rs`](crates/wingfc-core/src/math.rs) | **94.6%** | **95.0%** | **96.4%** | Vector cross/dot/norm, Quaternion multiplication/conjugate |
| **Arming Safety Interlocks** | [`safety.rs`](crates/wingfc-core/src/safety.rs) | **86.5%** | **77.8%** | **84.1%** | Configurable thresholds, high-throttle lockout, zero-surge |
| **Rate PID & FeedForward** | [`pid.rs`](crates/wingfc-core/src/pid.rs) | **84.5%** | **80.0%** | **82.8%** | Anti-windup clamping, dynamic FeedForward transient & decay |
| **Mode Matrix & Cascades** | [`mode.rs`](crates/wingfc-core/src/mode.rs) | **82.9%** | **68.4%** | **81.8%** | Multi-switch mode ranges, Heading Hold & Alt Hold cascades |
| **In-Flight Pot & Bank Tuning**| [`tuning.rs`](crates/wingfc-core/src/tuning.rs) | **85.0%** | **100.0%** | **84.0%** | Potentiometer deadbands, bank switching, gain persistence |
| **FlySky iBus Protocol** | [`protocol/ibus.rs`](crates/wingfc-core/src/protocol/ibus.rs) | **82.3%** | **71.4%** | **85.7%** | 16-bit inverse checksum, 14-channel unpacking, failsafe bits |
| **GPS & NMEA Navigation** | [`gps.rs`](crates/wingfc-core/src/gps.rs) | **79.6%** | **85.7%** | **84.8%** | NMEA GGA/RMC parsing, home latching, RTH bearing & error |
| **VQF AHRS** | [`ahrs.rs`](crates/wingfc-core/src/ahrs.rs) | **79.5%** | **100.0%** | **84.3%** | 6-DoF inclination filtering, 9-DoF mag fusion, rest detection |
| **DShot Motor Protocol** | [`dshot.rs`](crates/wingfc-core/src/dshot.rs) | **76.9%** | **66.7%** | **82.9%** | 16-bit packet encoding, nibble checksum, SPI MOSI bitstream |
| **Total `wingfc-core`** | **All Modules** | **88.6%** | **88.2%** | **89.9%** | **31 unit tests, 100% passing** |

### Production Firmware Matrix (All 8 Release Permutations)

```bash
# Check all 8 permutations and host tests in CI
./scripts/build.sh matrix check

# Build and package all 8 release binaries to dist/
./scripts/build.sh matrix build
```

| Binary Name | Receiver | Motor Protocol | Navigation (`nav`: Baro + Compass + GPS) |
| :--- | :--- | :--- | :--- |
| `wingfc-firmware-crsf-pwm` | CRSF / ELRS | PWM 50&ndash;400Hz | Disabled |
| `wingfc-firmware-crsf-pwm-nav` | CRSF / ELRS | PWM 50&ndash;400Hz | Enabled |
| `wingfc-firmware-crsf-dshot` | CRSF / ELRS | Digital DShot | Disabled |
| `wingfc-firmware-crsf-dshot-nav` | CRSF / ELRS | Digital DShot | Enabled |
| `wingfc-firmware-ibus-pwm` | FlySky iBus | PWM 50&ndash;400Hz | Disabled |
| `wingfc-firmware-ibus-pwm-nav` | FlySky iBus | PWM 50&ndash;400Hz | Enabled |
| `wingfc-firmware-ibus-dshot` | FlySky iBus | Digital DShot | Disabled |
| `wingfc-firmware-ibus-dshot-nav` | FlySky iBus | Digital DShot | Enabled |

---

## Community & Ecosystem Links

* **Main Website & Live Telemetry Hub:** [ferroxrc.com](https://ferroxrc.com)
* **Ground Station Companion OS:** [ferrox-rc/flysky-i6x-rs](https://github.com/ferrox-rc/flysky-i6x-rs)
* **Hardware Carrier Board Repository:** [ferrox-rc/wingfc-hardware](https://github.com/ferrox-rc/wingfc-hardware)
* **Community Discord:** [OpenI6X / Ferrox Discord](https://discord.gg/3vKfYNTVa2)

---

<p align="center">
  <sub>Part of the <a href="https://github.com/ferrox-rc">Ferrox-RC</a> Open-Source Avionics Initiative. Licensed under MIT.</sub>
</p>
