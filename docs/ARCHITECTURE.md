# WingFC-rs System Architecture & Design Specification

`wingfc-rs` is a bare-metal, high-performance, `#![no_std]` flight controller firmware and flight dynamics core written entirely in Rust for the ARM Cortex-M4F architecture (specifically targetting the Nordic nRF52840 SoC on the Seeed Studio XIAO nRF52840 Sense / Sense Plus, and custom derivative PCBs).

---

## 1. System Design Principles

1. **Deterministic Execution:** Zero heap allocations (`no_std`), zero garbage collection pauses, and hard real-time execution at 500Hz (2000&mu;s period).
2. **Hardware Watchdog Supervision:** 2000ms hardware watchdog timer (`WDT`) supervised directly by the 500 Hz flight loop.
3. **Zero-Cost Abstractions:** Build-time Cargo feature flags eliminate inactive protocols, algorithms, and drivers at compile time via dead-code elimination.
4. **Defense-in-Depth Safety:** Multi-layered arming interlocks, zero-surge failsafe recovery, and physical separation between core logic (`wingfc-core`) and peripheral drivers (`wingfc-firmware`).
5. **Modularity for Rapid Testing:** Pure flight control algorithms are isolated in `wingfc-core` and fully testable on host platforms via standard `cargo test` (achieving 88.4% line coverage and 89.8% region coverage across 21 unit tests).
6. **Zero-Code User Configuration:** End users configure aircraft hardware, protocols, flight dynamics, per-servo limits, and PID tunes using a friendly configuration file ([`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml)) without touching Rust source code or build files (see the [Aircraft Configuration Guide](file:///home/bryansouza/Repos/wingfc-rs/docs/CONFIG.md)).
7. **Visual Architectural Flowcharts:** Complete Mermaid.js state and dataflow diagrams are available in the [System Architecture & Control Flowcharts](file:///home/bryansouza/Repos/wingfc-rs/docs/FLOWCHARTS.md).

---

## 2. Hardware Architecture & Peripheral Allocation

Physical pinout allocations, wiring isolation guidelines, electrical specs, and peripheral conflict matrices are detailed in the [Pin Configuration & Hardware Allocation Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/PINOUT.md).

### Summary of Assigned On-Chip Blocks:
* **`PWM0` (ESCs 1 & 2):** Drives Motor 1 (`D3`/`P0.29`) and Motor 2 (`D8`/`P1.13`) for single-motor or differential-thrust twins. Supports 50..400 Hz analog PWM or digital DShot frames.
* **`PWM1` (Primary Servos 1..4):** Drives standard flight control surfaces (`D0`/`P0.02`, `D1`/`P0.03`, `D2`/`P0.28`, `D9`/`P1.14`).
* **`PWM2` (Secondary Servos 5..8):** Available on Seeed Studio XIAO nRF52840 Sense Plus (`D10`/`P1.15`, `D11`/`P0.15`, `D12`/`P0.19`, `D13`/`P1.01`) for auxiliary gear, pan/tilt, or extra control surfaces.
* **`TWISPI0` (Internal IMU):** Dedicated 400 kHz Fast-Mode EasyDMA I2C bus communicating with the onboard ST LSM6DS3TR-C 6-DoF sensor (`P0.07` SDA, `P0.27` SCL, `P1.08` Power Rail with `HighDrive`).
* **`TWISPI1` (External Sensors):** Dedicated 400 kHz I2C bus on castellated pads (`D4`/`P0.04` SDA, `D5`/`P0.05` SCL) for external QMC5883L/HMC5883L magnetometers and BMP280/SPL06 barometers.
* **`UARTE0` (RC Receiver & Telemetry):** Buffered EasyDMA UART engine servicing high-speed RC protocols (CRSF/ELRS @ 420 kBaud or FlySky iBus @ 115.2 kBaud) and bidirectional telemetry transmission (see the [Receiver & Telemetry Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/RECEIVERS.md)).
* **`UARTE1` (Autonomous GPS):** Secondary EasyDMA serial engine on Sense Plus (`D14`/`P0.09` RX, `D15`/`P0.10` TX) running the zero-allocation NMEA navigation parser (see the [GPS Navigation Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/GPS_NAVIGATION.md)).
* **Visual Telemetry:** Coordinated dual-LED system featuring the 3-color RGB LED (`P0.26`, `P0.30`, `P0.06`) and discrete User LED (`P0.17`).

---

## 3. Real-Time Flight Loop (500 Hz / 2ms Period)

Execution runs on Embassy's cooperative async executor with prioritized DMA event processing and a hard real-time ticker:

```
                  +---------------------------------------------------+
                  |            500 Hz Real-Time Loop (2ms)           |
                  +---------------------------------------------------+
                                            |
                                            v
                                  [Pet Hardware WDT]
                                            |
                                            v
                                 [Drain Latest RC Packet]
                                            |
                                            v
                                [Evaluate Failsafe Timeout]
                                            |
                                            v
                             [Update Arming Safety State Machine]
                                            |
                                            v
                             [Apply In-Flight Pot PID Tuning]
                                            |
                                            v
                             [Sample LSM6DS3TR IMU via I2C]
                                            |
                                            v
                           [Apply 8-Way Board Mounting Matrix]
                                            |
                                            v
                           [PT1 Gyro Harmonic Vibration Filter]
                                            |
                                            v
                       [Madgwick AHRS Orientation Estimation (6/9-DoF)]
                                            |
                                            v
                       [Inner/Outer PID Loop (Rate / Angle / FF)]
                                            |
                                            v
                                 [Multi-Airframe Mixer]
                                            |
                                            v
                       [Per-Servo Calibration (Min, Max, Trim, Rev)]
                                            |
                                            v
                       [Output Actuation: Servos (PWM1/2) & ESC (PWM0)]
```

### Control Pipeline Steps:
1. **Watchdog Servicing:** The 500 Hz task pets the 1000ms hardware watchdog (`wdt_handle.pet()`). Any hang or lockup exceeding 1000ms triggers an automated hardware MCU reset.
2. **RC Input & Channel Mapping:** Non-blocking reception from `UARTE0`. Channels are mapped according to [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml) (`roll`, `pitch`, `throttle`, `yaw`, `arm`, `mode`).
3. **Failsafe Monitoring:** If no valid RC packet is received within `failsafe_timeout_ms`, the aircraft enters `Failsafe` state (throttle cut to zero, surfaces trimmed for level glide).
4. **Arming Safety State Machine:** Enforces arm-switch thresholds, zero-throttle interlocks, and boot-time switch positions.
5. **In-Flight Dynamic Tuning:** Real-time adjustment of selected PID gains via designated transmitter sliders or pots.
6. **IMU Sampling & Transformation:** Accel and gyro vectors are read and transformed through the configured 8-way mounting orientation matrix (`default`, `cw90`, `cw180`, `cw270`, `flip`, etc.).
7. **Attitude Estimation & Control Loop:** The Madgwick AHRS filters gyro rates and computes pitch/roll attitude. Inner angular rate PID loops and outer attitude P loops generate normalized mixer demands.
8. **Airframe Mixing & Per-Servo Calibration:** Mixer maps demands across the selected topology (`elevon`, `elevon_diff_thrust`, `single_aileron_t_tail`, `dual_aileron_t_tail`, `single_aileron_v_tail`, `dual_aileron_v_tail`). Each output is individually clamped, trimmed, and inverted according to its `ServoConfig`.
9. **Physical Actuator Dispatch:** `PWM0` updates ESC pulse widths; `PWM1` (and `PWM2` on Sense Plus) updates servo pulse widths.
