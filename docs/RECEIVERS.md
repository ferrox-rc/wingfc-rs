# WingFC-rs Receiver & Telemetry Specification

This document details the RC receiver architecture, physical wiring, serial protocol engines, bidirectional CRSF telemetry formats, failsafe protection, and transmitter configuration for `wingfc-rs`.

---

## 1. Supported Receiver Protocols

`wingfc-rs` supports two high-speed serial protocols selected at build-time via Cargo feature flags or through the [`wingfc.yaml`](../wingfc.yaml) configuration file:

| Protocol | Feature Flag | Baud Rate | Framing | Duplex | Supported Hardware Systems |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **CRSF / ExpressLRS** | `--features crsf` | 420,000 baud | 8N1 | Full-Duplex (RX + Telemetry TX) | ExpressLRS (2.4 GHz &amp; 900 MHz), TBS Crossfire, TBS Tracer |
| **FlySky iBus** | `--features ibus` | 115,200 baud | 8N1 | Simplex (RX only) | FlySky AFHDS 2A receivers (FS-iA6B, FS-X6B, FS-A8S) |

> [!NOTE]
> Receiver features are mutually exclusive. Exactly one protocol (`crsf` or `ibus`) is compiled into each release artifact to eliminate unused interrupt handlers, minimize flash consumption, and guarantee zero runtime branch latency.

---

## 2. Hardware Pinout & Physical Wiring

All serial receiver communication is handled directly by the Nordic nRF52840's hardware `UARTE0` peripheral using non-blocking EasyDMA.

### Physical Pin Assignments

```
         Seeed Studio XIAO nRF52840 (Header View)
                  +-----------------------+
                  | [D0]             [5V] | <-- 5V VBUS / Battery BEC (4.5V - 5.5V)
                  | [D1]            [GND] | <-- Common Ground Reference (0V)
                  | [D2]           [3V3]  | <-- 3.3V Regulated Output (Max 200mA)
                  | [D3]            [D10] |
                  | [D4]             [D9] |
                  | [D5]             [D8] |
RC Telemetry TX ->| [D6]             [D7] | <- RC Receiver RX (UARTE0)
                  +-----------------------+
```

| XIAO Pin | nRF52840 Pin | Subsystem | Function / Role | Connected Receiver Pin |
| :--- | :--- | :--- | :--- | :--- |
| **`D7`** | `P1.12` | `UARTE0_RX` | Serial Channel Input | Receiver **TX** / Signal Out |
| **`D6`** | `P1.11` | `UARTE0_TX` | Serial Telemetry Output | Receiver **RX** / Telemetry In (CRSF only) |
| **`5V`** | `VBUS` | Power | Receiver Power Supply | Receiver **VCC / 5V** |
| **`GND`** | `GND` | Power | Common Ground Reference | Receiver **GND** |

---

### Wiring Example: ExpressLRS Receiver (Radiomaster RP1 / RP2, EP1 / EP2, BetaFPV Nano)

ExpressLRS receivers communicate at 420,000 baud and require both RX and TX lines for bidirectional channel control and telemetry:

```
+---------------------------+             +---------------------------+
|  Seeed Studio XIAO        |             |  ExpressLRS Receiver      |
|                           |             |                           |
|       D7 (P1.12 / RX) <---|-------------|-- TX (CH1 Output)         |
|       D6 (P1.11 / TX) ----|------------>|-- RX (CH2 Telemetry Input)|
|                    5V ----|------------>|-- 5V / VCC                |
|                   GND ----|-------------|-- GND                     |
+---------------------------+             +---------------------------+
```

> [!WARNING]
> Ensure the receiver's TX connects to the flight controller's RX (`D7`), and the receiver's RX connects to the flight controller's TX (`D6`). Do not connect TX-to-TX or RX-to-RX.

---

### Wiring Example: FlySky iBus Receiver (FS-iA6B, FS-X6B)

FlySky iBus operates in simplex mode at 115,200 baud over a single serial line:

```
+---------------------------+             +---------------------------+
|  Seeed Studio XIAO        |             |  FlySky iBus Receiver     |
|                           |             |                           |
|       D7 (P1.12 / RX) <---|-------------|-- iBus SERVO / S-PORT     |
|       D6 (P1.11 / TX)     | (Disconnected)|                         |
|                    5V ----|------------>|-- VCC (4.0V - 6.5V)       |
|                   GND ----|-------------|-- GND                     |
+---------------------------+             +---------------------------+
```

---

## 3. CRSF / ExpressLRS Technical Specification

The CRSF protocol operates at 420,000 baud with low latency, 16-channel resolution, and automatic telemetry downlinks.

### 3.1 Uplink: RC Channels Frame (`0x16`)

- **Frame Length:** 26 bytes total (`[0xC8] [0x18] [0x16] [22 bytes payload] [CRC8]`).
- **Channel Count:** 16 channels, encoded as 11-bit unsigned integers (0&ndash;2047).
- **CRSF Tick Range:** 172 ticks (minimum, 988 &mu;s) to 1811 ticks (maximum, 2012 &mu;s), with 992 ticks representing center (1500 &mu;s).
- **Conversion Equation:**
  ```text
  pulse_us = (ticks * 5 / 8) + 880
  ```
- **Checksum:** DVB-S2 CRC-8 polynomial (`0xD5`), initialized to `0x00`, calculated across `[TYPE]` and `[PAYLOAD]`.

---

### 3.2 Downlink: Bidirectional Telemetry Architecture

The telemetry engine in [`crates/wingfc-firmware/src/tasks/telemetry.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/tasks/telemetry.rs) operates on a 50 Hz round-robin slot scheduler. It streams real-time aircraft data back to the handset via `UARTE0_TX` (`D7` / `P0.11`):

```
+-----------------------------------------------------------------------------------+
|                            50 Hz Telemetry Slot Ticker                            |
+---------------------+---------------------+------------------+--------------------+
| Slot 0 (10 Hz)      | Slot 1 (10 Hz)      | Slot 2 (10 Hz)   | Slot 3 & 4 (10 Hz) |
| Vehicle Attitude    | Flight Mode String  | Battery Sensor   | GPS / Barometer    |
| (Type 0x1E)         | (Type 0x21)         | (Type 0x08)      | (Type 0x02 / 0x09) |
+---------------------+---------------------+------------------+--------------------+
```

All frames follow the standard CRSF structure:
```
[SYNC: 0xC8] [LEN: payload_len + 2] [FRAME_TYPE] [PAYLOAD: N bytes] [CRC8: 1 byte]
```

#### Frame Specifications

1. **Vehicle Attitude (`0x1E`):**
   - **Payload (6 bytes):**
     - Pitch: `i16` big-endian, radians &times; 10,000.
     - Roll: `i16` big-endian, radians &times; 10,000.
     - Yaw: `i16` big-endian, radians &times; 10,000.
   - **Handset Sensors:** Pitch, Roll, Yaw displayed in degrees on EdgeTX.

2. **Flight Mode String (`0x21`):**
   - **Payload (N &le; 16 bytes):** Null-terminated ASCII string indicating the active flight controller status:
     - `"DISARM"`: Vehicle disarmed, motors locked, ground pre-flight control check active.
     - `"ACRO"`: Rate mode active, angular velocity control with feedforward.
     - `"ANGLE"`: Self-leveling stabilized mode active.
     - `"FAILSAFE"`: RC connection lost, motor cut, neutral surfaces held.
     - `"REFUSED"`: Arming refused due to safety switch position or raised throttle.
   - **Handset Sensors:** Displayed as `Mode` on the transmitter home screen.

3. **Battery Sensor (`0x08`):**
   - **Payload (8 bytes):**
     - Voltage: `u16` big-endian in decivolts (0.1 V resolution).
     - Current: `u16` big-endian in deciapps (0.1 A resolution).
     - Capacity: `u24` big-endian in mAh.
     - Remaining Percentage: `u8` (0&ndash;100%).
   - **Handset Sensors:** `RxBt`, `Curr`, `Capa`, `Bat%`.

4. **GPS Navigation (`0x02`):**
   - **Payload (15 bytes):**
     - Latitude: `i32` big-endian, degrees &times; 10<sup>7</sup>.
     - Longitude: `i32` big-endian, degrees &times; 10<sup>7</sup>.
     - Ground Speed: `u16` big-endian, km/h &times; 10.
     - Ground Course: `u16` big-endian, degrees &times; 100.
     - Altitude MSL: `u16` big-endian, meters + 1,000 m offset.
     - Satellites: `u8` satellite count.
   - **Handset Sensors:** `GPS`, `GSpd`, `Hdg`, `Alt`, `Sats`.

5. **Barometer Altitude &amp; Vario (`0x09`):**
   - **Payload (4 bytes):**
     - Altitude MSL: `u16` big-endian, meters + 10,000 m offset.
     - Variometer: `i16` big-endian, vertical climb rate in cm/s.
   - **Handset Sensors:** `Alt`, `VSpd`.

---

## 4. FlySky iBus Technical Specification

The FlySky iBus protocol operates at 115,200 baud, 8 data bits, no parity, 1 stop bit (8N1).

### 4.1 Frame Structure

Each iBus packet consists of 32 bytes transmitted every 7.7 ms:
```
+--------------+--------------+-------------------------------+-------------------+
| Header (2B)  | Length (1B)  | 14 Channel Data Words (28B)   | Checksum (2B)     |
| 0x20 0x40    | 0x20 (32B)   | Ch1..Ch14 (u16 Little-Endian) | 16-bit Sum Check  |
+--------------+--------------+-------------------------------+-------------------+
```

- **Channel Values:** Direct microsecond pulse widths (1000 &mu;s to 2000 &mu;s, center 1500 &mu;s).
- **Checksum Calculation:**
  ```text
  checksum = 0xFFFF - sum(bytes[0..30])
  ```
  The received packet is validated by checking `checksum == u16::from_le_bytes(bytes[30..32])`.

---

## 5. Failsafe & Pre-Arm Safety Architecture

`wingfc-rs` enforces strict hardware-level and state-machine safety checks to prevent uncontrolled flight or accidental propeller injury.

### 5.1 Failsafe Timeout Detection

The receiver task timestamp is refreshed every time a cryptographically and checksum-valid packet is received. If no valid frame arrives within **500 ms** (`FAILSAFE_TIMEOUT`):

1. **Motor Disarm:** Throttle output is immediately forced to 1000 &mu;s (or DShot digital disarm command `0`).
2. **Neutral Flight Surfaces:** Control mixer commands `mix(0.0, 0.0, 0.0, 1000)`, holding aerodynamic surfaces neutral.
3. **PID Reset:** Pitch, roll, and yaw integral accumulators are zeroed to prevent wind-up upon reconnection.
4. **Visual Warning:** Onboard RGB LED shifts to **Slow Blink Red** (250 ms on, 250 ms off).
5. **Telemetry Warning:** Flight mode string changes to `"FAILSAFE"`, triggering transmitter audio alarms.

### 5.2 Visual Telemetry & LED Diagnostics Matrix

The onboard tri-color RGB LED indicates real-time link health, sensor calibration, and arming states at 20 Hz:

| LED Color | Visual Pattern | Frequency / Timing | Arming / Flight State | Meaning & Required Pilot Action |
| :--- | :--- | :--- | :--- | :--- |
| **White** | Fast Flash | 5 Hz (100ms ON / 100ms OFF) | `VisualStatus::Calibration` | **Gyro Stationary Calibration:** Measuring zero-rate bias at boot (250 samples). Keep aircraft flat and completely motionless. |
| **Yellow** | Fast Flash | 5 Hz (100ms ON / 100ms OFF) | `VisualStatus::EscCalibration` | **ESC Calibration Mode:** Throttle was high (&gt;1850 &mu;s) at boot. ESC endpoints calibration active (PWM only). |
| **Yellow** | Double Blink | 2 pulses every 1.0 s | `ArmingStatus::Refused` | **Arming Refused (Safety Lockout):** Arm switch engaged while throttle &gt; 1050 &mu;s, or arm switch was active during boot. Lower throttle and cycle arm switch. |
| **Red** | Slow Blink | 2 Hz (250ms ON / 250ms OFF) | `ArmingStatus::Failsafe` | **Failsafe / Link Lost:** No valid RC frames received within timeout (default: 500ms). Check receiver power, binding, and signal wiring on D7 (`P1.12`). |
| **Green** | Solid ON | Constant | `ArmingStatus::Disarmed` | **Disarmed & Ready:** Valid RC link active, gyro calibrated, motors disabled. Aircraft is safe to handle and ready to arm. |
| **Blue** | Solid ON | Constant | `ArmingStatus::ArmedAcro` | **Armed (Acro / Rate Mode):** Flight loop live, motors active, angular rate PID stabilization enabled. |
| **Cyan** | Solid ON | Constant | `ArmingStatus::ArmedAngle` | **Armed (Angle / Self-Leveling):** Flight loop live, motors active, attitude self-leveling stabilized flight enabled. |

---

### 5.3 Pre-Arm Safety Interlocks

Before motors can spin, the following conditions must be met simultaneously:

1. **Hardware Link Active:** Failsafe timeout must not be expired.
2. **Arm Switch Disarmed at Boot:** Arm switch must be initialized below `arm_switch_threshold_us` (default 1800 &mu;s) at system power-on. Arming will be refused (`REFUSED`) if powered up with the arm switch already active.
3. **Throttle Low Interlock:** Throttle stick must be strictly at or below `throttle_zero_threshold_us` (default 1050 &mu;s). If the throttle is raised, arming is locked out.
4. **Gyro Calibration Complete:** Stationary calibration must complete (indicated by White LED flashing).

---

## 6. EdgeTX / OpenTX Transmitter Setup Guide

### 6.1 Channel Mapping (AETR Order)

Configure your radio model with the standard **AETR** channel layout:

| Radio Channel | Control Surface | Function | Center | Travel Range |
| :--- | :--- | :--- | :--- | :--- |
| **CH1** | Roll / Aileron | Bank left / right | 1500 &mu;s | 1000 &ndash; 2000 &mu;s |
| **CH2** | Pitch / Elevator | Pitch nose up / down | 1500 &mu;s | 1000 &ndash; 2000 &mu;s |
| **CH3** | Throttle | Motor thrust command | 1000 &mu;s | 1000 &ndash; 2000 &mu;s |
| **CH4** | Yaw / Rudder | Rudder / nosewheel steering | 1500 &mu;s | 1000 &ndash; 2000 &mu;s |
| **CH5** | Auxiliary 1 | 2-Position Arm Switch | &mdash; | Low: 1000 &mu;s, High: 2000 &mu;s |
| **CH6** | Auxiliary 2 | Flight Mode Toggle | &mdash; | Low: Acro, High: Angle |

### 6.2 Arming Switch Configuration (CH5)

- In EdgeTX / OpenTX, assign a dedicated 2-position switch (e.g. `SA` or `SF`) to **CH5**.
- In the **MIXES** menu:
  - Disarmed position: Output = `-100%` (1000 &mu;s).
  - Armed position: Output = `+100%` (2000 &mu;s).

> [!TIP]
> In your radio settings, configure a "Switch Warning" on `CH5` so the transmitter beeps and warns you if the radio is powered on with the arm switch activated.

### 6.3 Flight Mode Switch Configuration (CH6)

- Assign a 2-position or 3-position switch (e.g. `SB`) to **CH6**:
  - `CH6 < 1500 &mu;s`: **Acro / Rates Mode** (Solid Blue LED, direct rate control).
  - `CH6 >= 1500 &mu;s`: **Angle / Stabilized Mode** (Solid Cyan LED, self-leveling).

---

### 6.4 Telemetry Discovery on EdgeTX

When using CRSF / ExpressLRS:

1. Power on the transmitter and aircraft.
2. Navigate to the **Model Settings &rarr; TELEMETRY** page on your handset.
3. Select **Discover new sensors**.
4. Within 2 seconds, the following telemetry items will appear:
   - `Ptch`: Aircraft pitch angle in degrees (&plusmn;90&deg;).
   - `Roll`: Aircraft roll angle in degrees (&plusmn;180&deg;).
   - `Yaw`: Aircraft yaw angle in degrees.
   - `Mode`: Active flight controller mode (`DISARM`, `ACRO`, `ANGLE`, `FAILSAFE`, `REFUSED`).
   - `RxBt`: Flight pack voltage.
   - `GPS`: Decimal coordinates and GPS status (if GPS enabled).
   - `GSpd`: Ground speed in km/h.
   - `Alt`: Barometric / GPS altitude in meters.

---

## 7. Troubleshooting & Diagnostics

| Symptom | Likely Cause | Recommended Action |
| :--- | :--- | :--- |
| **Slow Blink Red LED continuously** | Receiver not communicating or unpowered | Check wiring: Receiver TX must connect to `D6`. Verify receiver has power LED illuminated. |
| **Double Blink Yellow LED on arming** | Arming refused (Throttle raised or switch armed at boot) | Lower throttle stick to zero. Ensure arm switch is toggled down before attempting to arm. |
| **RC channels respond but no telemetry on radio** | `D7` (TX) pin disconnected or reversed | Connect receiver RX pin to XIAO `D7` (`P0.11`). Run "Discover new sensors" on transmitter. |
| **Channels twitch or jump unexpectedly** | Incompatible receiver baud rate or signal noise | Ensure firmware was compiled with the matching protocol (`--features crsf` for ELRS, `--features ibus` for FlySky). |
| **Motors do not spin when armed** | ESC throttle range mismatch | Run ESC calibration or verify `motor.pwm_frequency_hz` matches ESC specifications in `wingfc.yaml`. |
