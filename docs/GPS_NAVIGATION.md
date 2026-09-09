# GPS Navigation & Return-to-Home (RTH) Subsystem

This document provides technical documentation for the GPS navigation subsystem, NMEA protocol parser, coordinate projection mathematics, and Return-to-Home (RTH) guidance engine in `wingfc-rs`.

---

## 1. Overview & Architectural Role

The GPS navigation engine resides in the `#![no_std]` core crate ([`crates/wingfc-core/src/gps.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-core/src/gps.rs)). It enables autonomous position awareness, course and speed tracking, home point acquisition, and Return-to-Home navigation vectors.

```
                           GPS Subsystem Dataflow
  +--------------------+
  | External GPS (UBX) |
  | (u-blox M8N / M10) |
  +---------+----------+
            | 115,200 baud NMEA Stream ($..GGA, $..RMC)
            v
  +--------------------+
  |    UARTE1 RX       | (Physical Pin: P0.09 / D14 on XIAO Sense Plus, connects to GPS TX)
  +---------+----------+
            | Async EasyDMA Byte Stream
            v
  +--------------------+
  |    NmeaParser      | (Zero-allocation byte-by-byte streaming parser)
  | - XOR Checksum     |
  | - Fixed-point math |
  +---------+----------+
            | Validated GpsData (Lat, Lon, Alt, Speed, Course, Sats, HDOP)
            v
  +--------------------+
  |   GpsNavigation    |
  | - Home Latching    | (Criteria: >= 6 Sats, HDOP <= 2.5, Fix3D)
  | - Distance/Bearing | (Equirectangular Projection)
  | - RthGuidance      | (Heading Error, Safe Climb Floor)
  +---------+----------+
            |
            v
  +--------------------+
  | Flight Controller  | -> Injected into Autopilot / Stabilizer Loops
  +--------------------+
```

---

## 2. Hardware Requirements & Board Selection

### Standard XIAO vs. XIAO Sense Plus
* **Standard XIAO nRF52840 Sense:** Exposes 8 physical GPIO pins (`D0`&ndash;`D7`). Under the baseline flight controller architecture, all 8 pins are fully committed to Elevon Servos (`D0`, `D1`), Auxiliary Rudder (`D2`), ESC Motor (`D3`), External I2C Bus (`D4`, `D5`), CRSF Telemetry TX (`D6`), and RC Receiver Serial RX (`D7`). **Standard XIAO cannot connect to a GPS receiver.**
* **XIAO nRF52840 Sense Plus:** Provides extended castellated header pins (`D8`&ndash;`D11`) and exposes the secondary asynchronous hardware serial engine (**`UARTE1`**).

### Physical Pin Assignments

| Signal | Peripheral | nRF52840 Pin | Silkscreen | Electrical Spec |
| :--- | :--- | :--- | :--- | :--- |
| **Flight Controller RX** | `UARTE1` RX | `P0.09` | `D14` (NFC1) | Connects to GPS Module **TX** (3.3V LVCMOS Input, 115,200 Baud) |
| **Flight Controller TX** | `UARTE1` TX | `P0.10` | `D15` (NFC2) | Connects to GPS Module **RX** (3.3V LVCMOS Output, 115,200 Baud) |
| **GPS Power** | System 5V | &mdash; | `5V` (VBUS) | 5V DC Supply from Flight Battery BEC |
| **Ground** | Ground | &mdash; | `GND` | Common Ground Reference |

> [!IMPORTANT]
> **nRF52840 Hardware NFC Pin Reconfiguration:**
> Physical SoC pins `P0.09` and `P0.10` are configured as hardware Near Field Communication (NFC) antenna pins by default in silicon. To enable them as standard high-speed digital GPIOs, `wingfc-firmware` enables the `nfc-pins-as-gpio` feature in `embassy-nrf`. This disables the internal NFC clamping diodes at system startup.


### GPS Module Physical Wiring (Crossover Connection)

As with all UART serial connections, communication lines must be crossed over: Flight Controller RX connects to GPS Module TX, and Flight Controller TX connects to GPS Module RX.

```
+---------------------------+             +---------------------------+
|  XIAO Sense Plus (FC)     |             |  GPS Module (NMEA / UBX)  |
|                           |             |                           |
|      D14 (P0.09 / RX) <---|-------------|-- TX (Serial NMEA Out)    |
|      D15 (P0.10 / TX) ----|------------>|-- RX (Configuration In)   |
|                    5V ----|------------>|-- 5V / VCC Power Supply   |
|                   GND ----|-------------|-- GND Common Reference    |
+---------------------------+             +---------------------------+
```

> [!WARNING]
> **UART Crossover Rule:** Always wire Flight Controller RX (`D14` / `P0.09`) to GPS Module **TX**, and Flight Controller TX (`D15` / `P0.10`) to GPS Module **RX**. Never connect TX-to-TX or RX-to-RX.

---

## 3. Zero-Allocation NMEA Streaming Parser (`NmeaParser`)

The parser operates without dynamic heap allocation (`alloc`) and is fully deterministic:
* **Internal Static Buffer:** Uses a 96-byte fixed ring buffer (`buf: [u8; 96]`).
* **Frame Synchronization:** Resets index upon encountering `$` start delimiter.
* **Non-Blocking Ingestion:** Ingests individual bytes via `parser.feed_byte(byte: u8) -> Option<&GpsData>`.
* **Checksum Verification:** Computes the 8-bit XOR sum of all ASCII characters between `$` and `*`, comparing against the two-digit hex value following `*`. Corrupted sentences are immediately rejected.

### Supported Sentences
1. **`$..GGA` (Global Positioning System Fix Data):**
   * Fields: UTC Time, Latitude (`ddmm.mmmm`), N/S indicator, Longitude (`dddmm.mmmm`), E/W indicator, Fix Quality (0=Invalid, 1=3D, 2=DGPS), Satellites Tracked, HDOP, Altitude Above Mean Sea Level (MSL).
2. **`$..RMC` (Recommended Minimum Specific GPS Data):**
   * Fields: UTC Time, Validity Status (`A`=Active, `V`=Void), Latitude, Longitude, Speed Over Ground (knots &rarr; converted to meters/second: 1 knot = 0.514444 m/s), Course Over Ground (degrees).

---

## 4. Navigation & Coordinate Mathematics

### Coordinate Conversion
NMEA outputs coordinates in degrees and decimal minutes (`ddmm.mmmm` / `dddmm.mmmm`). The parser converts them to signed decimal degrees:
* Latitude: Degrees = `dd`, Minutes = `mm.mmmm` &rarr; Decimal = Degrees + (Minutes / 60.0). If South, negate.
* Longitude: Degrees = `ddd`, Minutes = `mm.mmmm` &rarr; Decimal = Degrees + (Minutes / 60.0). If West, negate.

### Equirectangular Flat-Earth Projection
For tactical operations within 100 km of the launch site, spherical Earth trigonometry can be projected onto a localized planar grid without loss of precision (&lt;0.1% angular error):

* Mean Latitude: &phi;<sub>mid</sub> = (&phi;<sub>1</sub> + &phi;<sub>2</sub>) / 2
* Delta Latitude: &Delta;&phi; = (&phi;<sub>2</sub> &minus; &phi;<sub>1</sub>) &times; (&pi; / 180)
* Delta Longitude: &Delta;&lambda; = (&lambda;<sub>2</sub> &minus; &lambda;<sub>1</sub>) &times; (&pi; / 180)
* Projected Coordinates:
  * x = &Delta;&lambda; &times; cos(&phi;<sub>mid</sub>)
  * y = &Delta;&phi;
* Surface Distance:
  * Distance (m) = R<sub>earth</sub> &times; &radic;(x<sup>2</sup> + y<sup>2</sup>), &nbsp;&nbsp; where R<sub>earth</sub> = 6,371,000 meters
* True Bearing to Home:
  * Bearing (&deg;) = atan2(x, y) &times; (180 / &pi;), normalized to [0&deg;, 360&deg;)

### Autonomous Home Latching Criteria
Before Return-to-Home can be engaged, the flight controller must establish a trustworthy reference origin. The home position latches automatically when all three conditions are satisfied:
1. `fix_type == Fix3D` or `FixDgps`
2. `satellites >= min_satellites` (default: &ge; 6 satellites)
3. `hdop <= max_hdop` (default: &le; 2.5 HDOP)

Once latched, the home latitude, longitude, and ground altitude MSL remain locked for the duration of the flight.

### Return-to-Home (RTH) Guidance Generation
When RTH is engaged (either via RC switch or failsafe link loss), `GpsNavigation::calculate_rth_guidance` generates real-time steering vectors:
* **Distance to Home:** Line-of-sight horizontal distance in meters.
* **Bearing to Home:** True course direction toward the launch coordinates.
* **Heading Error:** Angular difference between current ground track (`course_deg`) and target bearing:
  * Heading Error = Bearing &minus; Course, wrapped to the range [&minus;180&deg;, +180&deg;].
  * Negative error commands a left turn; positive error commands a right turn.
* **Safe Target Altitude:** Altitude MSL target = Home Altitude MSL + Safe Return Floor (default: +30.0m AGL). Ensures the plane clears trees and structures before turning home.

---

## 5. Aircraft Configuration (`wingfc.yaml`)

GPS navigation parameters are configured in [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml):

```yaml
# Set target board to the expanded variant
board: xiao-sense-plus

navigation:
  enabled: false               # Master switch for sensor fusion suite
  barometer: false             # Vertical Kalman filter (BMP280/DPS310)
  compass: false               # 9-DoF AHRS tilt-compensated heading (QMC5883L)
  gps: true                    # Enable NMEA GPS parsing & RTH guidance

  # Return-to-Home (RTH) Parameters
  rth:
    min_satellites: 6          # Minimum satellites to latch home
    max_hdop: 2.5              # Maximum HDOP acceptable for home latch
    safe_altitude_agl_m: 30.0  # Autonomous return climb altitude Above Ground Level

  hard_deck_agl_m: 15.0        # Hard deck floor (m AGL)
  max_ceiling_agl_m: 120.0     # Legal airspace ceiling (m AGL)
```

### Build System Validation Rules
When running `./scripts/build.sh` or `./scripts/build.sh check`:
* If `navigation.gps: true` (or `navigation.enabled: true`) is specified while `board: xiao-sense` is selected, the build script halts with an explicit error explaining that the standard XIAO board lacks the necessary secondary UART breakout pins.
* If `board: xiao-sense-plus` is selected, the script builds with the `gps` feature enabled, generating appropriate constants in [`crates/wingfc-firmware/src/config.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/config.rs).
