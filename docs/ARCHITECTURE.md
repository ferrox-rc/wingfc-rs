# WingFC-rs System Architecture & Design Specification

`wingfc-rs` is a bare-metal, high-performance, `#![no_std]` flight controller firmware and flight dynamics core written entirely in Rust for the ARM Cortex-M4F architecture (specifically targetting the Nordic nRF52840 SoC on the Seeed Studio XIAO nRF52840 Sense, and custom derivative PCBs).

---

## 1. System Design Principles

1. **Deterministic Execution:** Zero heap allocations (`no_std`), zero garbage collection pauses, and hard real-time execution at 500Hz (2000&mu;s period).
2. **Zero-Cost Abstractions:** Build-time Cargo feature flags eliminate inactive protocols, algorithms, and drivers at compile time via dead-code elimination.
3. **Defense-in-Depth Safety:** Multi-layered arming interlocks, zero-surge failsafe recovery, and physical separation between core logic (`wingfc-core`) and peripheral drivers (`wingfc-firmware`).
4. **Modularity for Rapid Testing:** Pure flight control algorithms are isolated in `wingfc-core` and fully testable on host platforms via standard `cargo test`.
5. **Zero-Code User Configuration:** End users configure aircraft hardware, protocols, flight dynamics, and PID tunes using a friendly configuration file ([`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml)) without touching Rust source code or build files (see [Aircraft Configuration Guide](file:///home/bryansouza/Repos/wingfc-rs/docs/CONFIG.md)).

---

## 2. Hardware Architecture & Pin Configuration System

Board pin definitions and peripheral bindings are centralized in [`crates/wingfc-firmware/src/pins.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/pins.rs) via the `BoardPins` abstraction. For detailed electrical specs, peripheral conflict analysis, and expansion wiring diagrams, see the dedicated [Pin Configuration & Hardware Allocation Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/PINOUT.md).

### Base Target: Seeed Studio XIAO nRF52840 Sense
* **MCU:** Nordic Semiconductor nRF52840 (ARM Cortex-M4F with hardware FPU @ 64 MHz).
* **Memory:** 1,024 KB Flash, 256 KB RAM.
* **Onboard IMU:** STMicroelectronics LSM6DS3TR (3-axis Gyroscope + 3-axis Accelerometer).

```
                            Seeed XIAO nRF52840 Sense
                         +-----------------------------+
   Elevon Left  (PWM0) --| D0 (P0.02)         (P0.06)  |-- Blue LED (Telemetry)
   Elevon Right (PWM0) --| D1 (P0.03)         (P0.30)  |-- Green LED (Telemetry)
   Aux Servo    (PWM0) --| D2 (P0.28)         (P0.26)  |-- Red LED (Telemetry)
   ESC Motor 1  (PWM1) --| D3 (P0.29)         (P0.07)  |-- IMU SCL (TWISPI0)
   Available / Ext SDA --| D4 (P0.04)         (P0.27)  |-- IMU SDA (TWISPI0)
   Available / Ext SCL --| D5 (P0.05)         (P0.15)  |-- IMU Power Rail Enable
   RC RX (EasyDMA UARTE)-| D6 (P1.00)         (3.3V)   |
   Available / GPIO    --| D7 (P0.11)         (GND)    |
                         +-----------------------------+\n```

### Pinout Reference:
| Peripheral | Pin | Description | Driver / Subsystem |
| :--- | :--- | :--- | :--- |
| **Servo 1** | `P0.02` (D0) | Left Elevon (50..400Hz PWM, 1000..2000 &mu;s) | `ServoPwmDriver` (PWM0, Ch 0) |
| **Servo 2** | `P0.03` (D1) | Right Elevon (50..400Hz PWM, 1000..2000 &mu;s) | `ServoPwmDriver` (PWM0, Ch 1) |
| **Servo 4** | `P0.28` (D2) | Auxiliary Servo (50..400Hz PWM, 1000..2000 &mu;s) | `ServoPwmDriver` (PWM0, Ch 2) |
| **Motor 1** | `P0.29` (D3) | Motor ESC (50..400Hz PWM or DShot) | `EscPwmDriver` (PWM1) |
| **RC Input** | `P1.00` (D6) | EasyDMA UART RX (CRSF @ 420kBaud / iBus @ 115.2kBaud) | `UarteRx` (UARTE0) |
| **IMU SCL** | `P0.07` | I2C Clock (400 kHz Fast Mode) | `Twim` (TWISPI0) |
| **IMU SDA** | `P0.27` | I2C Data (400 kHz Fast Mode) | `Twim` (TWISPI0) |
| **IMU PWR** | `P0.15` | Internal Sensor Rail Enable (Active HIGH) | `Output` (GPIO) |
| **LED Red** | `P0.26` | Active-LOW Visual Telemetry Indicator | `RgbLedDriver` (GPIO) |
| **LED Green**| `P0.30` | Active-LOW Visual Telemetry Indicator | `RgbLedDriver` (GPIO) |
| **LED Blue** | `P0.06` | Active-LOW Visual Telemetry Indicator | `RgbLedDriver` (GPIO) |

### Future GPS Hardware Requirement: XIAO nRF52840 Sense Plus
Integrating autonomous GPS navigation (NMEA / UBX) requires upgrading from the standard XIAO Sense to the **Seeed Studio XIAO nRF52840 Sense Plus** variant:
- **Secondary UART Allocation (`UARTE1`):** `UARTE0` is dedicated to high-speed RC reception (`P1.00`). GPS reception and configuration require a second asynchronous serial engine (`UARTE1` RX/TX) operating at 115,200 baud.
- **Pin Availability:** The standard XIAO form factor exposes only 8 external GPIOs (`D0`&ndash;`D7`), which are exhausted once 4 PWM channels, RC input, and isolated external I2C (magnetometer/barometer) are wired. The **Sense Plus** breaks out additional pins (`D8`, `D9`, `D10`, `D11` and solder pads), providing dedicated lines for GPS serial, analog battery voltage sensing, and buzzer alarms.

---

## 3. Real-Time Flight Loop (500 Hz / 2ms Period)

Execution runs on Embassy's cooperative async executor with prioritized DMA event processing and a hard real-time ticker:

```
                  +---------------------------------------------------+
                  |            500 Hz Real-Time Loop (2ms)           |
                  +---------------------------------------------------+
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
                             [Sample LSM6DS3TR IMU via I2C]
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
                                  [Elevon Airframe Mixer]
                                            |
                                            v
                       [Output Actuation: Servos (50..400Hz) & ESC (50..400Hz/DShot)]
```

### Control Pipeline Steps:
1. **RC Ingestion:** Non-blocking drain of the async channel queue populated by the EasyDMA UART receiver task.
2. **Failsafe Monitoring:** If the duration since the last valid packet exceeds 500ms, the system enters failsafe mode (motors cut to zero throttle, control surfaces held level).
3. **Safety State Machine:** Evaluates arming switch (CH5), mode switch (CH6), throttle position, and failsafe flag to determine the operational state.
4. **IMU Sampling:** Samples gyroscope (rad/s) and accelerometer (m/s<sup>2</sup>) data over 400kHz I2C.
5. **Harmonic Vibration Filtering:** Raw gyroscope rates pass through single-pole PT1 low-pass filters with a 90Hz cutoff frequency (DT = 0.002s) to reject motor and propeller vibrations before entering AHRS or PID calculations:
   <br>&alpha; = (2&pi; &middot; f<sub>c</sub> &middot; &Delta;t) / (1 + 2&pi; &middot; f<sub>c</sub> &middot; &Delta;t), &nbsp;&nbsp; y<sub>k</sub> = y<sub>k-1</sub> + &alpha;(x<sub>k</sub> - y<sub>k-1</sub>)
6. **Attitude Estimation (Madgwick AHRS):**
   * Computes quaternion attitude updates with dynamic G-rejection: acceleration corrections are only integrated when total G-force is within [0.65g, 1.35g] (1.0 &plusmn; 0.35g). In violent aerobatic maneuvers, pure gyroscope integration prevents centripetal acceleration distortion.
   * When `nav` / `compass` is enabled, 9-DoF magnetometer data is fused to eliminate yaw drift and lock heading to magnetic North.
7. **PID Controllers with FeedForward:**
   * **Acro / Rate Mode:** Command rates directly from pilot stick inputs up to 600&deg;/s Roll, 200&deg;/s Pitch, 100&deg;/s Yaw.
   * **Angle / Stabilized Mode:** Outer proportional loop converts pitch/roll angle error into target rates, which are tracked by the inner rate PID.
   * **FeedForward (FF):** Stick derivative is filtered through a 45Hz PT1 filter and injected directly into the actuator command, eliminating phase lag on aggressive stick maneuvers.
   * **Anti-Windup:** Accumulators are clamped to `max_integral` (0.40), and reset immediately when disarmed.
8. **Airframe Mixer (Elevon):**
   <br>Servo<sub>1</sub> = clamp(1500 &minus; Roll + Pitch), &nbsp;&nbsp; Servo<sub>2</sub> = clamp(1500 &minus; Roll &minus; Pitch)
   <br>Linear throttle output is clamped to 1000..2000 &mu;s (or mapped to 0..2047 for DShot).

---

## 4. Actuator Outputs, PWM Drivers & Protocols

The nRF52840 PWM peripheral is driven by a 16MHz clock prescaled to 1MHz (`Prescaler::Div16`), yielding an exact 1.0 &mu;s timer tick resolution.

### A. Servo PWM Driver (`ServoPwmDriver` on PWM0)
* **Channels:** 3 channels (`D0` Left Elevon, `D1` Right Elevon, `D2` Auxiliary).
* **Configurable Refresh Rate:** 50 Hz to 400 Hz (default 50 Hz).
* **Hardware Warning & Limits:**
  * **Analog Servos:** Must strictly operate at 50 Hz (max 60 Hz). Frequencies above 60 Hz cause analog servos to rapidly overheat, chatter violently, and burn out their internal drive motors.
  * **High-Speed Digital Servos:** High-end digital servos natively accept 200 Hz, 250 Hz, 333 Hz, or up to 400 Hz, significantly reducing phase lag on elevon control surfaces.
  * **Physical Upper Bound:** 400 Hz maximum. At 400 Hz, the total frame period is 2500 &mu;s, leaving a 500 &mu;s low-level off-time following a 2000 &mu;s high pulse. Frequencies &gt;400 Hz leave insufficient falling-edge detection time.

### B. ESC Motor Driver (`EscPwmDriver` on PWM1)
* **Analog PWM (`not(feature = "dshot")`):**
  * **Configurable Refresh Rate:** 50 Hz to 400 Hz (default 400 Hz).
  * **Fixed-Wing Compatibility Warning:** While multirotor ESCs (BLHeli_S/32, AM32) run at 400 Hz without issue, many budget or legacy fixed-wing airplane ESCs only tolerate 50 Hz or 60 Hz. If an analog ESC refuses to arm or beeps error codes, set `motor.pwm_frequency_hz: 50`.
  * **Early PWM Startup (Step 0):** Hardware PWM is initialized to 1000 &mu;s immediately upon boot before I2C/UART initialization, allowing analog ESCs to start arming sequences promptly.
  * **Calibration Pass-Through Mode:** If the pilot powers the aircraft on with full throttle (&gt;1900 &mu;s), the controller enters pass-through calibration mode (fast blinking yellow LED) until the throttle is lowered to zero (&lt;1050 &mu;s).
* **DShot Digital Protocol (`feature = "dshot"`):**
  * Transmits 16-bit digital packets with 4-bit XOR checksum over DMA.
  * Range: 0 (Disarm / Motor Stop), 48 (Minimum Throttle), 2047 (Maximum Throttle).
  * Eliminates analog endpoint calibration completely.

---

## 5. Navigation Suite Architecture (`feature = "nav"`)

The unified `nav` feature combines the barometer and compass subsystem:

### A. Vertical Kalman Filter (`baro`)
Fuses high-rate (500Hz) vertical accelerometer predictions with lower-rate (20-50Hz) barometric altitude measurements:
* **State Vector:** [z, z&#775;]<sup>T</sup> (altitude in meters, vertical velocity in m/s).
* **Covariance Propagation:** Accommodates accelerometer variance Q<sub>accel</sub> and barometric sensor noise R<sub>baro</sub>.
* Yields smooth, low-latency vertical speed estimation for energy management, climb monitoring, and altitude safety.

### B. Altitude Safety Controller (`baro`) & Regulatory Legality
* **Hard Deck Floor:** If the aircraft breaches the hard floor (e.g., &lt;15 m AGL) with negative vertical speed, it overrides pilot commands to apply positive climb pitch and climb throttle.
* **Statutory Ceiling Limiter (120 m AGL):** Under FAA 14 CFR &sect; 107.51 / 49 U.S.C. &sect; 44809 (USA) and EASA Regulation (EU) 2019/947 UAS.OPEN.010 (Europe), sUAS flight is strictly limited to 120 meters (400 feet) AGL in open category airspace. The ceiling limiter prevents breaching this regulatory limit by automatically pitching the nose down and throttling back. Exceeding 120m AGL without prior civil aviation authority airspace authorization is illegal and introduces severe mid-air collision hazards with manned aviation.
* **Stall Recovery:** Detects rapid sink rates at level flight attitudes and pitches the nose down while commanding full throttle to recover airspeed.

### C. 9-DoF Madgwick AHRS (`compass`)
* Fuses 3-axis magnetometer readings with the gravity-aligned attitude quaternion.
* Rotates measured magnetic vectors to the Earth frame (h = q &otimes; [0, m] &otimes; q<sup>*</sup>) and calculates gradient descent corrections against Earth magnetic field components b<sub>x</sub>, b<sub>z</sub>.
* Guarantees true heading lock with zero drift during prolonged straight-line navigation.

### D. Autonomous GPS Navigation & Return-to-Home (`gps`)
* **Zero-Allocation NMEA 0183 Parser:** High-efficiency, `#![no_std]` streaming parser consuming GGA and RMC sentences byte-by-byte with XOR checksum verification.
* **Telemetry Decoded:** Latitude, longitude, altitude MSL, speed over ground (SOG), course over ground (COG), satellite count, HDOP, and fix quality (2D, 3D, DGPS).
* **Automatic Home Point Latching:** Latches initial launch coordinates and baseline altitude upon achieving a high-precision 3D lock (&ge; 6 satellites, HDOP &le; 2.5).
* **Equirectangular Flat-Earth Projection:** Calculates real-time distance and bearing to the latched home point with &lt;0.1% distortion over tactical flight ranges (&lt;100 km).
* **Return-to-Home (RTH) Guidance Vector:** Emits continuous bearing-to-home, heading error relative to ground track (&plusmn;180&deg;), and safe climb altitude targets (Home MSL + 30m AGL).
* **Hardware Requirement:** Demands the **Seeed Studio XIAO nRF52840 Sense Plus** variant with secondary asynchronous serial engine (`UARTE1` on pins `D9`/`D10`), as standard XIAO exhausts all 8 GPIOs.
* **Dedicated Specification:** For full protocol parsing specifications, mathematical models, and wiring diagrams, see the [GPS Navigation & Return-to-Home Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/GPS_NAVIGATION.md).

---

## 6. Visual Telemetry (RGB LED) States

Visual telemetry is driven by an asynchronous 20Hz task monitoring the global `STATUS_WATCH` channel:

| Visual Pattern | State | Description |
| :--- | :--- | :--- |
| **Fast Blink Yellow** (100ms on, 100ms off) | `EscCalibration` | ESC Throttle Range Calibration Mode (PWM only) |
| **Fast Blink White** (100ms on, 100ms off) | `Calibration` | Stationary Gyroscope Bias Calibration in progress |
| **Solid Green** | `Disarmed` | System safe, sensors calibrated, ready for arming |
| **Solid Blue** | `ArmedAcro` | Aircraft armed in Acro / Rates mode |
| **Solid Cyan** | `ArmedAngle` | Aircraft armed in Angle / Stabilized self-leveling mode |
| **Double Blink Yellow** (1-sec cycle) | `ArmingRefused` | Arming refused: switch armed at boot or throttle raised |
| **Slow Blink Red** (250ms on, 250ms off) | `Failsafe` | Failsafe active: loss of RC link or sensor failure |

---

## 7. Safety Interlocks & Pre-Arm Rules

1. **Pre-Arm Switch Interlock:** If the arm switch (CH5) is already in the armed position when the flight controller powers on, arming is refused (`RefusedSwitchArmedAtBoot`) until the pilot cycles the switch to DISARM.
2. **Throttle Zero Interlock:** Arming is refused (`RefusedThrottleRaised`) if throttle is &gt;1050 &mu;s when attempting to arm.
3. **Zero-Surge Failsafe Recovery:** When link recovery occurs following a failsafe, the controller will NOT re-arm mid-air if the pilot's throttle stick is high; it requires safe, intentional pilot input to clear.
4. **Hand Autolaunch Safety:** Finger-safety motor delay (200ms) ensures the motor does not spin until the aircraft has physically left the pilot's hand, followed by a 400ms throttle ramp to prevent torque roll.

---

## 8. Real-Time Telemetry & Blackbox Logging Architecture

1. **Bench & In-Flight Debugging (SEGGER RTT):**
   * Operates over SWD (`SWDIO`/`SWDCLK`) directly into host tools (`probe-rs`) with sub-microsecond RAM write overhead, avoiding UART buffer stalls in the 500Hz loop.
   * Multi-channel RTT ring buffers isolate string logs (`defmt`), raw binary telemetry (Channel 1: 500Hz 28-byte blackbox struct), and command injection.
2. **Autonomous Onboard Blackbox & Companion Co-Processor:**
   * Because the bare Seeed XIAO nRF52840 lacks onboard high-speed NOR flash, an SD slot, and available GPIOs, custom derivative PCBs incorporate a companion co-processor (RP2040 / ESP32) linked via SPI or SWD.
   * The companion chip handles FatFS SD card block writes and wear-leveling asynchronously, preserving the nRF52840 500Hz flight loop jitter-free (<1 &mu;s timing variation).
   * For complete hardware layouts, packet binary structures, and flight log analysis tools, see the [SEGGER RTT & Blackbox Logging Architecture Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/RTT_BLACKBOX_LOGGING.md).
