# wingfc-rs

<p align="center">
  <img src="assets/logo.svg" alt="Ferrox-RC Emblem" width="100" height="100" />
</p>

<p align="center">
  <strong>Deterministic, Bare-Metal Rust Flight Controller for Sub-250g Autonomous UAVs & Flying Wings</strong><br>
  <em>The airborne flight companion in the <a href="https://ferroxrc.com">Ferrox-RC</a> avionics ecosystem</em>
</p>

<p align="center">
  <a href="https://ferroxrc.com/#wingfc"><img src="https://img.shields.io/badge/Status-In%20Active%20Bench%20Development-00E5FF.svg?style=flat-square" alt="Status" /></a>
  <a href="https://ferroxrc.com"><img src="https://img.shields.io/badge/Ecosystem-ferroxrc.com-E05A47.svg?style=flat-square" alt="Website" /></a>
  <a href="https://github.com/ferrox-rc/flysky-i6x-rs"><img src="https://img.shields.io/badge/Companion%20OS-flysky--i6x--rs-00E676.svg?style=flat-square" alt="Ground Companion" /></a>
  <img src="https://img.shields.io/badge/License-MIT%20%2F%20Apache--2.0-blue.svg?style=flat-square" alt="License" />
</p>

---

## ⚡ Overview

`wingfc-rs` brings hard real-time, deterministic `#![no_std]` Rust directly into autonomous airborne avionics. Specifically tailored for micro fixed-wing platforms, sub-250g FPV wings, and agile autonomous UAVs.

Powered by the **Seeed Studio XIAO nRF52840** (ARM Cortex-M4F @ 64 MHz with single-cycle hardware FPU) and the non-blocking **Embassy** cooperative async runtime, `wingfc-rs` replaces bloated monolithic flight stacks with deterministic lockstep execution.

### Key Architecture Highlights

- **500 Hz Lockstep Control Loop**: Paced by hardware timer interrupts and IMU DRDY signals for jitter-free rate and attitude control.
- **Hardware FPU Madgwick AHRS**: Sub-millisecond single-cycle floating-point quaternion estimation fusing accelerometer and gyroscope measurements.
- **Bi-Directional Telemetry Downlink**: Continuous i-BUS & CRSF downlink telemetry streaming battery voltage, RSSI, and attitude angles straight to your [`flysky-i6x-rs`](https://github.com/ferrox-rc/flysky-i6x-rs) ground station.
- **Integer-Paced Elevon Mixing**: High-resolution Catmull-Rom throttle curves and differential elevon actuation designed specifically for delta-wing aerodynamics.
- **Fail-Safe & Return-To-Level**: Autonomous wing leveling and hardware-level throttle cutoff on loss of RF sync.

---

## 🛰️ Ecosystem Architecture

`wingfc-rs` is engineered as the airborne counterpart to [`flysky-i6x-rs`](https://github.com/ferrox-rc/flysky-i6x-rs), establishing an end-to-end bare-metal Rust control link:

```
┌────────────────────────────────┐         AFHDS 2A / 260 Hz          ┌────────────────────────────────┐
│       FlySky FS-i6X TX         │ ◄────────────────────────────────► │     Airborne Receiver          │
│        (flysky-i6x-rs)         │        Bidirectional RF            │     (FS-iA6B / Fli14+)         │
└────────────────────────────────┘                                    └───────────────┬────────────────┘
                                                                                      │ i-BUS / CRSF
                                                                                      ▼
                                                                      ┌────────────────────────────────┐
                                                                      │           wingfc-rs            │
                                                                      │   Seeed XIAO nRF52840 (64MHz)  │
                                                                      │   • 500 Hz Embassy Loop        │
                                                                      │   • Hardware FPU Madgwick      │
                                                                      │   • Twin Elevon Servo PWM      │
                                                                      └────────────────────────────────┘
```

---

## 📅 Release Roadmap & Status

`wingfc-rs` is currently undergoing active bench, hardware-in-the-loop (HIL), and oscilloscope validation on physical silicon. The full repository source code and pre-built UF2 drag-and-drop binaries will be published here upon completion of hardware flight tests.

- [x] Cooperative Embassy async runtime & sensor HAL
- [x] 6-DOF IMU acquisition & hardware interrupt DRDY pacing
- [x] Hardware FPU Madgwick quaternion filter & attitude estimator
- [x] Dual elevon differential mixer & integer curve math
- [ ] Hardware-in-the-loop (HIL) aerodynamic bench runs
- [ ] First flight-test telemetry log publication
- [ ] Public source release & UF2 drag-and-drop binaries

---

## 🌐 Community & Links

- **Main Website & Live Telemetry Hub**: [ferroxrc.com](https://ferroxrc.com)
- **Ground Station OS**: [ferrox-rc/flysky-i6x-rs](https://github.com/ferrox-rc/flysky-i6x-rs)
- **Community Discord**: [OpenI6X / Ferrox Discord](https://discord.gg/3vKfYNTVa2)

---

<p align="center">
  <sub>Part of the <a href="https://github.com/ferrox-rc">Ferrox-RC</a> Open-Source Avionics Initiative. Licensed under MIT or Apache-2.0.</sub>
</p>
