# High-Speed Telemetry, RTT, and Blackbox Logging Architecture

This document details the real-time telemetry and logging architecture for `wingfc-rs`, explaining how **SEGGER Real-Time Transfer (RTT)** and **Serial Wire Debug (SWD)** are utilized on the bench today, and how dedicated co-processor hardware and SD card storage will be integrated into the upcoming custom PCB.

---

## 1. Real-Time Transfer (RTT) & SWD on the Bench

### What is SEGGER RTT?
SEGGER Real-Time Transfer (RTT) is an ultra-low-overhead bidirectional communication protocol that operates directly through the ARM Cortex-M **Serial Wire Debug (SWD)** pins (`SWDIO`, `SWDCLK`). 

Unlike traditional UART serial logging or USB CDC-ACM virtual COM ports:
* **Zero Extra Pins:** RTT utilizes the existing 2-wire SWD debug interface. No GPIO or hardware UART engines are consumed.
* **Zero Loop Jitter (<1 &mu;s execution):** The Cortex-M4F CPU simply performs an `O(1)` memory copy into a ring buffer allocated in SRAM. The CPU does **not** wait for shift registers, baud clock divisors, or USB bus packet acknowledgments.
* **Asynchronous Memory Access by Debug Probe:** The host debug probe (e.g., J-Link, ST-Link, or CMSIS-DAP via `probe-rs`) reads and drains the ring buffer memory addresses directly over SWD while the 500Hz flight loop runs at full speed uninterrupted.

```mermaid
graph LR
    subgraph "Host Workstation"
        P["probe-rs / RTT Host Client"]
        D["Defmt Decoder & Demuxer"]
        G["Live Telemetry Plotter / File Logger"]
    end

    subgraph "SWD Physical Link"
        SWD["SWDIO + SWDCLK (10-20 MHz)"]
    end

    subgraph "Nordic nRF52840 SoC (Flight Controller)"
        RAM["256 KB SRAM Control Block"]
        FL["500 Hz Real-Time Flight Loop"]
    end

    FL -->|"Memcopy (<1 &mu;s)"| RAM
    RAM -->|"Hardware DMA over SWD"| SWD
    SWD --> P
    P --> D
    D --> G
```

### Using the Seeed Studio XIAO DebugMate

The recommended bench interface for `wingfc-rs` is the **Seeed Studio XIAO DebugMate**. It combines an RP2040-based CMSIS-DAP debugger and a USB-to-UART bridge in a cradle form factor:

* **Solderless Pogo-Pin Connection:** The XIAO module snaps into the cradle without soldering header wires. Spring-loaded pogo pins establish contact with the underside test pads: `SWDIO`, `SWDCLK`, `3V3`, and `GND`.
* **Single-Cable Workstation Link:** A single USB-C cable to the DebugMate supplies 3.3V target power, flashes firmware over SWD, decodes live RTT telemetry, and dumps QSPI flash logs.
* **Clock Rate Configuration:** In `.cargo/config.toml`, the SWD speed is set to `--speed 1000` (1000 kHz) to ensure noise-free transfers across the pogo-pin interface.

```
+--------------------------------------------------------------+
| Host PC (Linux / macOS / Windows)                            |
| Running cargo run, probe-rs, or scripts/blackbox.py          |
+--------------------------------------------------------------+
                               |
                        [USB-C Cable]
                               |
                               v
+--------------------------------------------------------------+
| Seeed Studio XIAO DebugMate                                  |
|   Onboard RP2040 CMSIS-DAP v2 Debugger (USB VID 0x0D28)      |
+--------------------------------------------------------------+
                               |
                [Spring-Loaded Pogo Pins]
               (SWDIO, SWDCLK, 3V3, GND)
                               |
                               v
+--------------------------------------------------------------+
| Seeed Studio XIAO nRF52840 Sense                             |
|   ARM Cortex-M4F Running WingFC-rs Firmware                  |
|   - defmt telemetry via RTT Channel 0                        |
|   - In-RAM BBL_SWD_MAILBOX for offloading QSPI NOR flash     |
+--------------------------------------------------------------+
```

### Bench Setup Workflow with `probe-rs`

During bench testing, motor vibration audits, and sensor calibration:

1. **Mount to DebugMate:** Snap the XIAO nRF52840 into the DebugMate cradle and plug the USB-C cable into your workstation.
2. **Launch Live RTT Telemetry Stream:**
   ```bash
   # Flashes target and opens live defmt RTT terminal (configured in .cargo/config.toml)
   cargo run --release

   # Or invoke probe-rs directly
   probe-rs run --chip nRF52840_xxAA --speed 1000 target/thumbv7em-none-eabihf/release/wingfc-firmware
   ```
3. **Deferred Formatting (`defmt`):**
   `wingfc-rs` uses `defmt` with `defmt-rtt`. Strings are interned at compile time: only 1-to-2 byte token IDs and raw binary values are transmitted over SWD. The host `probe-rs` utility decodes the stream using the local `.elf` symbol table, providing high-bandwidth logging with minimal binary footprint.
4. **Offload QSPI Blackbox Logs over SWD:**
   When testing on the bench, flight logs stored in the onboard Puya P25Q16H flash can be downloaded through the DebugMate without switching USB cables or opening a COM port:
   ```bash
   # Read flash status and session counts via DebugMate SWD
   python3 scripts/blackbox.py info --swd

   # Download flash and convert to flight.bbl and flight.csv over SWD
   python3 scripts/blackbox.py dump flight --swd

   # Erase logged sectors over SWD before the next test run
   python3 scripts/blackbox.py erase --swd
   ```

---

## 2. Multi-Channel RTT Protocol Specification

The RTT control block supports up to 16 independent bidirectional channels. In `wingfc-rs`, RTT is structured into three dedicated channels:

| Channel | Direction | Name | Buffer Size | Format | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0** | Target &rarr; Host | `defmt` | 1024 B | Defmt Tokenized Binary | System status, arming alerts, and boot diagnostics |
| **1** | Target &rarr; Host | `blackbox` | 2048 B | Packed Binary Struct | Real-time state snapshots for high-rate vibration and PID analysis |
| **2** | Host &rarr; Target | `tuning_cli`| 256 B | Compact ASCII / Bytecode | Live PID and filter tuning commands from the bench |

### High-Rate Streaming Frame (Channel 1)
During bench testing, a compact 28-byte binary frame can be streamed over RTT Channel 1:

```rust
#[repr(C, packed)]
pub struct RttBlackboxFrame {
    pub timestamp_us: u32,    // 4 bytes: Loop monotonic timestamp
    pub gyro_raw: [i16; 3],    // 6 bytes: Gyro X, Y, Z (0.1 deg/s resolution)
    pub accel_raw: [i16; 3],   // 6 bytes: Accel X, Y, Z (0.01 m/s^2 resolution)
    pub rc_setpoint: [i16; 3], // 6 bytes: Commanded Roll, Pitch, Throttle
    pub servo_out: [i16; 2],   // 4 bytes: Elevon 1 & 2 microsecond outputs
    pub state_flags: u16,      // 2 bytes: Arming state, flight mode, failsafe flag
} // Total: 28 bytes per cycle
```

Because the SWD link sustains 800 KB/s to 2.5 MB/s, streaming at loop speed consumes less than 2% of available debug link bandwidth. For autonomous in-flight recording away from the bench probe, see [Onboard QSPI Blackbox Flight Data Recorder](BLACKBOX.md).

---

## 3. Persistent Storage: Onboard QSPI Flash vs. MicroSD

While RTT is ideal for real-time bench monitoring over SWD, in-flight flight logging requires non-volatile storage:

1. **Onboard 2 MB QSPI NOR Flash (Current XIAO Implementation):**
   WingFC-rs records armed flight sessions directly to the Seeed Studio XIAO's onboard Puya P25Q16H 2 MB NOR flash at 50 Hz. This provides 21.6 minutes of autonomous flight recording per cycle without consuming external GPIOs. Complete frame format specifications, offload instructions, and Betaflight Blackbox Explorer workflows are documented in [Onboard QSPI Blackbox Flight Data Recorder](BLACKBOX.md).
2. **Upcoming Custom PCB Architecture (Co-Processor & SD Card):**
   To support multi-hour continuous recording without SD card write latency spikes impacting the primary flight loop, future custom carrier hardware offloads FatFS and SDIO operations to a dedicated auxiliary MCU.

---

## 4. Custom PCB Architecture: Dedicated Co-Processor & SD Card Subsystem

To provide hours of continuous high-rate logging without degrading the primary flight controller's real-time deterministic performance, the upcoming custom hardware layout incorporates an auxiliary co-processor:

```mermaid
graph TD
    subgraph "Custom Integrated PCB"
        subgraph "Primary Flight Core (nRF52840)"
            FC["ARM Cortex-M4F @ 64 MHz"]
            IMU["Onboard 6-DoF / 9-DoF IMU"]
            RC["CRSF / iBus Receiver"]
            PWM["Elevon Servos & ESC PWM"]
        end

        subgraph "High-Speed Inter-Chip Link"
            LINK["High-Speed SPI Bus / SWD Link (16-32 MHz)"]
        end

        subgraph "Telemetry & Logging Co-Processor (RP2040 / ESP32)"
            COP["Auxiliary MCU (Dual Cortex-M0+ / Xtensa)"]
            DMA["SDIO / FatFS DMA Engine"]
            WIFI["Optional Wireless Telemetry Link"]
        end

        subgraph "Storage & Radios"
            SD["MicroSD Card Slot (FAT32 / exFAT)"]
            FLASH["16MB QSPI High-Speed Flash"]
            ANT["Wireless Antenna (Wi-Fi / BLE Telemetry)"]
        end
    end

    IMU --> FC
    RC --> FC
    FC --> PWM

    FC -->|"Direct DMA Stream (Zero CPU Load)"| LINK
    LINK --> COP
    COP --> DMA
    DMA --> SD
    DMA --> FLASH
    COP -.-> WIFI
    WIFI -.-> ANT
```

### Architecture Advantages:
1. **Zero Real-Time Penalty on the Flight Controller:**
   SD card write operations routinely encounter latency spikes (100 to 250 ms when an SD card internal controller performs wear-leveling or block allocation). Offloading file system operations to a companion co-processor completely insulates the primary flight loop from SD latency spikes.
2. **Dedicated Multi-Hour Storage:**
   The co-processor buffers incoming stream frames in RAM and flushes them to the MicroSD card using 4-bit high-speed SDIO DMA. A standard 32 GB MicroSD card stores hundreds of flight hours.
3. **Wireless Ground Station Bridge:**
   An ESP32 or wireless-capable co-processor can simultaneously serve as a high-speed Wi-Fi or Bluetooth bridge, transmitting live telemetry to a ground station tablet or laptop while writing the primary log to the SD card.

---

## 5. Dedicated Documentation Reference

For practical flight logging guides, frame byte definitions, and offloading instructions, see:

* [Onboard QSPI Blackbox Flight Data Recorder](BLACKBOX.md): Details the 32-byte binary frame format, USB/SWD dump procedures via `scripts/blackbox.py`, and Betaflight Blackbox Explorer airplane mixer setup.
* [Pin Configuration & Hardware Allocation Specification](PINOUT.md): Details pin assignments and electrical characteristics for the nRF52840 QSPI bus on `P0.20` through `P0.25`.

