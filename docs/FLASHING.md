# WingFC-rs Firmware Deployment & Flashing Guide

This document details the memory layout, binary artifact formats, and supported flashing methods for the Seeed Studio XIAO nRF52840 Sense flight controller.

---

## 1. Hardware Memory Map & Sector Layout

The Seeed Studio XIAO nRF52840 Sense features an ARM Cortex-M4F MCU with 1,024 KB Flash and 256 KB RAM. The factory bootloader reserves space for the Nordic Semiconductor S140 SoftDevice (BLE stack) and the Adafruit/Seeed UF2 bootloader.

```
0x0000_0000 +-----------------------------------------------+
            |  Nordic S140 SoftDevice v7.3.0 & MBR (156 KB)  | (Protected / Preserved)
0x0002_7000 +-----------------------------------------------+
            |                                               |
            |  WingFC-rs Application Firmware (792 KB)      | (FLASH : ORIGIN = 0x27000)
            |  - Vector Table & Reset Handler (0x27101)     |
            |  - Embedded Flight Controller Logic           |
            |                                               |
0x000E_D000 +-----------------------------------------------+
            |  Adafruit/Seeed UF2 Bootloader (44 KB)        | (Protected / Preserved)
0x000F_8000 +-----------------------------------------------+
            |  Bootloader Settings & MBR Parameter Storage  | (32 KB)
0x0010_0000 +-----------------------------------------------+
```

### The `memory.x` Linker Specification
To coexist with the factory SoftDevice without overwriting it, [`crates/wingfc-firmware/memory.x`](../crates/wingfc-firmware/memory.x) configures the application origin:

```ld
MEMORY
{
  /* Space reserved for Nordic S140 SoftDevice v7.3.0 */
  FLASH : ORIGIN = 0x27000, LENGTH = 0xED000 - 0x27000
  RAM   : ORIGIN = 0x20006000, LENGTH = 0x20040000 - 0x20006000
}
```

* **Application Start:** `0x00027000`
* **Entry Point / Reset Vector:** `0x00027101`
* **Vector Table:** Located at the start of application flash (`0x27000`), loaded by the bootloader at startup.

---

## 2. Artifact Formats: Why Intel HEX (`.hex`) vs `.bin` vs `.uf2` vs `.elf`

When compiling WingFC-rs, four binary artifacts are produced in the `dist/` directory:

| Format | File Extension | Address Aware? | Best For | Safety Level |
| :--- | :--- | :---: | :--- | :--- |
| **Intel HEX** | `.hex` | **Yes** (Absolute) | J-Link (`nrfjprog`), `probe-rs download` | **Highest** (Prevents SoftDevice overwrites) |
| **UF2** | `.uf2` | **Yes** (Tagged Blocks) | USB Mass-Storage Drag-and-Drop | **High** (Bootloader enforces boundary) |
| **Raw Binary** | `.bin` | **No** (Flat Dump) | Legacy programmers, serial DFU | **Medium** (Requires explicit base address) |
| **ELF** | `.elf` | **Yes** (Full DWARF) | GDB debugging, `probe-rs run` with RTT | **High** (Development & defmt logs) |

### 1. Intel HEX (`.hex`) &mdash; The Preferred Standard
An Intel HEX file is an ASCII file of hexadecimal records that **explicitly encodes target physical memory addresses**:
```text
:020000022000DC                        <-- Record 02: Sets segment base to 0x00020000
:10700000000004200171020049BB0200...    <-- Record 00: Writes 16 bytes starting at 0x7000
```
* **Destination Address:** `0x20000 + 0x7000 = 0x00027000`.
* **Safe Sector Erase:** When programming with `nrfjprog --program firmware.hex --sectorerase --reset`, the programmer reads the target address directly from the HEX file and only erases the application sectors starting at `0x27000`. It is **physically impossible** to accidentally overwrite the SoftDevice (`0x00000`&ndash;`0x26FFF`) or bootloader.

### 2. Raw Binary (`.bin`) &mdash; Flat Byte Stream
A `.bin` file contains only raw bytes without any file headers or memory destination addresses:
* Byte `0` in the file corresponds to the lowest address linked (`0x27000`).
* **Why it worked:** Because `memory.x` linked all pointers, branch targets, and the vector table to absolute addresses &ge; `0x27000`, flashing the binary into offset `0x27000` runs smoothly.
* **Risk:** If a tool flashes the `.bin` without specifying `--base-address 0x27000`, it will overwrite `0x00000000`, corrupting the SoftDevice and causing a permanent crash.

### 3. UF2 (`.uf2`) &mdash; Zero-Tool Drag & Drop
UF2 (USB Flashing Format) breaks firmware into 512-byte blocks. Each block includes:
* Magic numbers recognizing the format
* Target physical flash address (starting at `0x27000`)
* Target Board Family ID (`0xADA52840` for Nordic nRF52840)
* Payload data (256 bytes)

---

## 3. Automated Flashing with `scripts/build.sh` / `scripts\build.bat`

The WingFC builder provides cross-platform automated flashing for Linux, macOS, and Windows (`scripts/build.sh` or `scripts\build.bat`).

Before any hardware flashing occurs, the builder automatically performs a pre-flash configuration check (`cmd_check`), prints your aircraft configuration and hardware advisories, and requests explicit user confirmation to proceed:

```text
Confirm configuration and proceed with build & flash to hardware? [y/N]: 
```

Once confirmed, the builder automatically recompiles release firmware with your confirmed configuration and flashes the target using either auto-detection or your selected method.

### Command Syntax
```bash
# Auto-detect connected programmer or mounted drive (Linux/macOS)
./scripts/build.sh flash

# Windows Command Prompt or PowerShell:
scripts\build.bat flash

# Automatically confirm prompt (useful for CI / non-interactive automation):
./scripts/build.sh flash -y
scripts\build.bat flash -y

# Explicitly select flashing method
./scripts/build.sh flash --method uf2           # USB Mass-Storage Drag-and-Drop (Windows: scripts\build.bat flash --method uf2)
./scripts/build.sh flash --method nrfjprog      # Nordic J-Link SWD via Intel HEX
./scripts/build.sh flash --method dfu           # adafruit-nrfutil serial DFU
./scripts/build.sh flash --method probe-rs-hex  # SWD Intel HEX download via probe-rs
./scripts/build.sh flash --method probe-rs-run  # SWD run with live defmt RTT logging
```

### Config File Settings (`wingfc.yaml`)
You can lock in your default method in `wingfc.yaml`:
```yaml
flashing:
  method: auto          # options: auto, uf2, nrfjprog, dfu, probe-rs-hex, probe-rs-run
  port: auto            # optional serial port for DFU (e.g., /dev/ttyACM0 or COM3)
  mount: auto           # optional mount path for UF2 drive (e.g., /run/media/$USER/XIAO-SENSE or D:\)
```

---

## 4. Step-by-Step Flashing Workflows

### Method A: USB Drag-and-Drop (UF2 Bootloader) &mdash; No Tools Required
1. Connect the Seeed Studio XIAO nRF52840 Sense to your PC via USB-C.
2. Quickly **double-tap the tiny reset button** (next to the USB-C port).
3. The board LED will pulse green, and a USB mass-storage drive named **`XIAO-SENSE`** will appear on your system (e.g., `/run/media/$USER/XIAO-SENSE/` on Linux, `D:\` on Windows).
4. Run:
   ```bash
   ./scripts/build.sh flash --method uf2    # Windows: scripts\build.bat flash --method uf2
   ```
   *(Or manually copy: `cp dist/wingfc-firmware-*.uf2 /run/media/$USER/XIAO-SENSE/` or Windows: `copy dist\wingfc-firmware-*.uf2 D:\`)*
5. The bootloader automatically writes the flash, unmounts the drive, and boots into WingFC.

### Method B: Nordic J-Link SWD (`nrfjprog`) &mdash; Professional Debugging
Requires an external J-Link programmer connected to the SWD test pads (`SWDIO`, `SWDCLK`, `GND`, `3V3` on the underside of the XIAO):

```bash
# Build firmware
./scripts/build.sh

# Flash Intel HEX with sector erase (preserves SoftDevice & Bootloader)
nrfjprog --program dist/wingfc-firmware-*.hex --sectorerase --reset
```
> [!IMPORTANT]
> Always use `--sectorerase`. Never use `--chiperase`, as `--chiperase` will erase the factory Nordic SoftDevice and UF2 bootloader.

### Method C: Serial DFU (`adafruit-nrfutil`)
For boards flashed over a USB virtual COM port:
```bash
# Generate DFU package from Intel HEX
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application dist/wingfc-firmware-*.hex dfu_pkg.zip

# Upload over serial
adafruit-nrfutil dfu serial --package dfu_pkg.zip -p /dev/ttyACM0 -b 115200
```

### Method D: `probe-rs` SWD
For CMSIS-DAP or J-Link probes:
```bash
# Download Intel HEX directly:
probe-rs download --chip nRF52840_xxAA --format hex dist/wingfc-firmware-*.hex

# Or run with live defmt RTT logging:
probe-rs run --chip nRF52840_xxAA dist/wingfc-firmware-*.elf
```

---

## 5. Manual Cargo Aliases & Direct Tool Extraction

If building directly via Cargo without `scripts/build.sh`, use the following commands:

```bash
# 1. Compile firmware for iBus or CRSF
cargo fw-build-ibus
# or
cargo fw-build-crsf

# 2. Extract self-addressing Intel HEX from ELF (Recommended)
llvm-objcopy -O ihex target/thumbv7em-none-eabihf/release/wingfc-firmware wingfc-rs.hex

# 3. Flash via nrfjprog
nrfjprog --program wingfc-rs.hex --sectorerase --reset
```
