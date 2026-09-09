# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-09-08

### 🚀 Features & Architecture Improvements
- **Unified 8-Servo Architecture & Sequential Nomenclature:** Expanded servo management to 8 fully independent, sequentially indexed outputs (`servo1` through `servo8`) across `PWM1` (channels 0..3) and `PWM2` (channels 0..3 on Sense Plus).
- **Dual-LED Visual Telemetry:** Added support for the discrete User/Charge indicator LED (`P0.17`) alongside the 3-color RGB LED. Non-GPS builds feature RC connection status, heartbeat, and packet streaming pulses. GPS builds provide satellite acquisition cadence, 3D fix strobe, and solid home lock indication.
- **Configurable Multi-Channel Actuator Architecture:** 
  - Dedicated `PWM0` to ESC throttle control for up to 2 motors with differential thrust support.
  - Dedicated `PWM1` as the primary 4-channel servo driver for standard and Sense Plus boards.
  - Dedicated `PWM2` as an expansion 4-channel servo driver on Seeed Studio XIAO nRF52840 Sense Plus.
  - Standardized all PWM pin drive outputs to `OutputDrive::Standard`.
- **In-Flight Dynamic PID Potentiometer Tuning:** Added real-time PID parameter adjustment via transmitter RC channels with deadband filtering and range scaling.
- **Hardware Watchdog Timer (WDT):** Integrated a 1000ms hardware watchdog in `main.rs` actively supervised by the 500 Hz flight loop.
- **Granular Per-Servo Endpoint & Sub-Trim Calibration:** Added individual `min_us`, `max_us`, `subtrim_us`, and `reversed` configurations in `wingfc.yaml`.
- **Multi-Airframe Mixing Matrix:** Expanded flight core mixer to support 6 airframe topologies: `elevon`, `elevon_diff_thrust`, `single_aileron_t_tail`, `dual_aileron_t_tail`, `single_aileron_v_tail`, and `dual_aileron_v_tail`.
- **8-Way Board Mounting Orientation Matrix:** Added sensor transform matrices for all 8 orthogonal mounting orientations (`default`, `cw90`, `cw180`, `cw270`, `flip`, `flip_cw90`, `flip_cw180`, `flip_cw270`).
- **Configurable Outer Angle P-Gain:** Exposed `angle_p_gain` in `wingfc.yaml` for tuning attitude self-leveling responsiveness.
- **Configurable RC Channel Selection:** Added explicit channel mapping in `wingfc.yaml` (`roll`, `pitch`, `throttle`, `yaw`, `arm`, `mode`) supporting various transmitter switch/pot layouts.
- **Idiomatic Embassy Async Drivers:** Upgraded UART receiver to double-buffered DMA driver and transitioned shared state channels to `ThreadModeRawMutex`.
- **UART Communication & Crossover Verification:** Audited all documentation and drivers to strictly enforce proper UART TX/RX crossovers and pin mappings (`D6`/`P1.11` TX, `D7`/`P1.12` RX on `UARTE0`; `D14`/`P0.09` RX, `D15`/`P0.10` TX on `UARTE1`).

## [0.1.2] - 2026-09-07

### 🚀 Features
- Add DShot digital motor protocol and zero-cost compile-time feature selection
- Complete Phase 4: Production release matrix, sizing verification, and CI/CD validation
- Refactor and document complete release matrix for all 8 firmware permutations

## [0.1.1] - 2026-09-05

### ⚙️ Miscellaneous Tasks
- Transition to cargo-workspace inheritance at v0.1.1 across all crates

### 🚀 Features
- Add GPS navigation subsystem, NMEA parser, RTH guidance, and documentation
- Add configurable arming and throttle safety thresholds, update documentation and release matrix
- Add GitHub Actions CI/CD pipeline and clean up clippy linter warnings
- Implement bidirectional CRSF telemetry protocol and asynchronous DMA broadcast task
- Add Receiver & Telemetry specification doc and update release matrix with measured binary sizes
- Added uf2conv to scripts

## [0.1.0] - 2026-09-04

### 💼 Other Changes
- Initial commit: High-performance Rust flight controller workspace with `#![no_std]` core, Madgwick AHRS, Acro Rate PID, and Embassy firmware
- Configure workspace cargo aliases and enable default host testing for `wingfc-core`

### 🚀 Features
- Add full FlySky iBus receiver protocol parser, 16-bit checksum verification, unified RcChannels abstraction, and unit tests
- Complete Phase 2: Dual multi-frequency PWM drivers, LSM6DS3TR I2C driver, EasyDMA UART receiver with CRSF/iBus, and gyro vibration filtering
- Implement build-time Cargo features for protocol selection (`crsf` vs `ibus`) with zero-cost dead-code elimination
- Complete Phase 3: Arming safety interlocks, RGB LED visual telemetry, zero-surge failsafe recovery, and safety unit tests
- Implement early ESC PWM startup (Step 0) and high-throttle ESC range calibration pass-through mode
- Implement and unit test FeedForward (FF) Rate PID, Hand Autolaunch, 9-DoF Madgwick AHRS, Vertical Kalman Filter, and Altitude Safety controllers
- Complete Phase 5: YAML aircraft configuration system, build automation, pinout architecture, PWM frequency tuning, and airspace legality warnings
