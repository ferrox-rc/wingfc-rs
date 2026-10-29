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
  <a href="https://github.com/ferrox-rc/wingfc-hardware"><img src="https://img.shields.io/badge/Hardware-wingfc--hardware-blueviolet.svg?style=flat-square" alt="Carrier Hardware" /></a>
  <img src="https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square" alt="License" />
</p>

---

## ⚡ Overview

`wingfc-rs` delivers deterministic, hard real-time `#![no_std]` Rust execution for micro fixed-wing aircraft, sub-250g FPV flying wings, and autonomous UAVs.

Operating on the **Seeed Studio XIAO nRF52840** (ARM Cortex-M4F @ 64 MHz with single-cycle hardware FPU) and the cooperative **Embassy** async runtime, `wingfc-rs` replaces bloated monolithic flight stacks with an architecture optimized specifically for fixed-wing aerodynamics.

### Key Architecture Highlights

* **3-Tier Multi-Rate Architecture:**
  * **1.666 kHz IMU Acquisition:** Dedicated hardware interrupt-paced sampling on `TWISPI0` with 8:1 boxcar downsampling (`BoxcarAccumulator8`) providing sinc low-pass rejection of motor harmonics.
  * **208.33 Hz Flight Control Loop:** Event-paced 4.8 ms control step running VQF AHRS attitude estimation, angular rate PIDs, and elevon mixing, updating autonomous hardware `PWM0/1/2` EasyDMA registers directly.
  * **50 Hz Background Sensors & Telemetry:** External navigation sensors on `TWISPI1`, bidirectional CRSF / ELRS telemetry, and onboard QSPI blackbox logging.
* **VQF Quaternion AHRS:** Versatile Quaternion-based Filter (Laidig & Seel) with strict decoupling of inclination (roll/pitch) from heading, inertial acceleration low-pass filtering, online gyro bias estimation, and magnetic disturbance rejection.
* **Native QSPI NOR Flash Blackbox:** Autonomous recording to the onboard 2 MB Puya P25Q16H flash at 50 Hz. Stores 32-byte binary frames with live roll/pitch PID terms (`[P, I, D]`), gyro rates, accelerations, actuator pulses, and CRC-8 integrity checks, compatible with the open-source Betaflight Blackbox Explorer.
* **Zero-Allocation NMEA GPS Engine:** High-speed streaming parser with automatic launch point latching, return-to-home navigation, and vertical Kalman filter altitude control.
* **Fail-Safe & Motor Safety:** Multi-layered arming interlocks, zero-surge failsafe recovery, and autonomous wing leveling on loss of RF sync.

---

## 🛰️ Ecosystem Architecture

`wingfc-rs` is engineered as the airborne counterpart to [`flysky-i6x-rs`](https://github.com/ferrox-rc/flysky-i6x-rs), establishing an end-to-end bare-metal Rust control link:

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

## 📅 Release Roadmap & Status

`wingfc-rs` has completed active bench validation and hardware-in-the-loop testing on physical silicon. The full repository source code and pre-built UF2 drag-and-drop binaries will be published here upon completion of final outdoor flight trials.

- [x] Cooperative Embassy async runtime & sensor HAL
- [x] 3-tier multi-rate architecture (1.666 kHz IMU & 8:1 decimation to 208.33 Hz)
- [x] VQF AHRS quaternion orientation estimator
- [x] Onboard 2 MB QSPI blackbox flight recorder (Betaflight Explorer support)
- [x] Autonomous navigation suite (Baro Vertical KF, tilt-compensated compass, GPS RTH)
- [x] Complete hardware build matrix & CI/CD pipeline
- [ ] First outdoor flight-test telemetry log publication
- [ ] Public source release & UF2 drag-and-drop binaries

---

## 🌐 Community & Links

* **Main Website & Live Telemetry Hub:** [ferroxrc.com](https://ferroxrc.com)
* **Ground Station OS:** [ferrox-rc/flysky-i6x-rs](https://github.com/ferrox-rc/flysky-i6x-rs)
* **Hardware Carrier Board Repository:** [ferrox-rc/wingfc-hardware](https://github.com/ferrox-rc/wingfc-hardware)
* **Community Discord:** [OpenI6X / Ferrox Discord](https://discord.gg/3vKfYNTVa2)

---

<p align="center">
  <sub>Part of the <a href="https://github.com/ferrox-rc">Ferrox-RC</a> Open-Source Avionics Initiative. Licensed under MIT.</sub>
</p>
