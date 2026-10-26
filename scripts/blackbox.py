#!/usr/bin/env python3
"""
WingFC Blackbox Log Offload & Analysis Tool.

Communicates with WingFC-rs flight controller over either:
  1. XIAO DebugMate SWD (`probe-rs` via in-RAM `BBL_SWD_MAILBOX`), or
  2. Native USB CDC-ACM serial port
to inspect, download, and decode high-rate 50 Hz binary flight logs from onboard QSPI NOR flash.
"""

import argparse
import binascii
import csv
import math
import os
import struct
import subprocess
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

FRAME_SIZE = 32
MAGIC_BYTE = 0xBB
VERSION_BYTE = 0x01

LOG_START_ADDR = 0x004000
SWD_MAILBOX_MAGIC = 0x57464342
SWD_CMD_NONE = 0
SWD_CMD_ERASE_USED = 1
SWD_CMD_ERASE_ALL = 2
SWD_CMD_READ_CHUNK = 3

SWD_STATUS_IDLE = 0
SWD_STATUS_BUSY = 1
SWD_STATUS_ERR_ARMED = 2
SWD_STATUS_ERR_IO = 3

SWD_BUF_BYTES = 4096


def crc8_itu(data: bytes) -> int:
    """Computes CRC-8 with polynomial 0x07 (ATM/ITU), matching firmware."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def find_wingfc_usb_port():
    """Returns native WingFC USB CDC-ACM port if connected, else None."""
    if not serial:
        return None
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        # Exclude CMSIS-DAP / DebugMate UART bridge (0x0d28:0x0204)
        if p.vid == 0x0D28 and p.pid == 0x0204:
            continue
        if "cmsis-dap" in desc:
            continue
        if (p.vid == 0x1209 and p.pid == 0x0001) or "wingfc" in desc or "wingfc" in mfg:
            return p.device
    return None


def should_use_swd(args) -> bool:
    """Determines whether to communicate via XIAO DebugMate SWD (probe-rs) or USB CDC."""
    if getattr(args, "swd", False):
        return True
    if getattr(args, "port", None):
        return False
    return find_wingfc_usb_port() is None


def find_elf_path(custom_elf=None) -> str:
    """Locates the compiled wingfc-firmware ELF binary to resolve BBL_SWD_MAILBOX."""
    if custom_elf and os.path.isfile(custom_elf):
        return custom_elf

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(repo_root, "target", "thumbv7em-none-eabihf", "debug", "wingfc-firmware"),
        os.path.join(repo_root, "target", "thumbv7em-none-eabihf", "release", "wingfc-firmware"),
    ]
    existing = [p for p in candidates if os.path.isfile(p)]
    if not existing:
        print(
            "Error: Compiled firmware ELF not found. Build or flash the firmware first with 'cargo run'.",
            file=sys.stderr,
        )
        sys.exit(1)
    # Pick most recently modified ELF
    existing.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return existing[0]


def resolve_symbol_address(elf_path: str, symbol_name: str) -> int:
    """Extracts a symbol's virtual address directly from a 32-bit little-endian ELF file."""
    with open(elf_path, "rb") as f:
        data = f.read()

    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise RuntimeError(f"Unsupported ELF format in {elf_path}")

    e_shoff = struct.unpack_from("<I", data, 32)[0]
    e_shentsize, e_shnum, _ = struct.unpack_from("<HHH", data, 46)

    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        sh_name, sh_type, sh_flags, sh_addr, sh_offset, sh_size, sh_link, sh_info, sh_addralign, sh_entsize = (
            struct.unpack_from("<IIIIIIIIII", data, off)
        )
        sections.append((sh_type, sh_offset, sh_size, sh_link, sh_entsize))

    target_bytes = symbol_name.encode("ascii")
    # SHT_SYMTAB == 2
    for sh_type, sh_offset, sh_size, sh_link, sh_entsize in sections:
        if sh_type == 2 and sh_entsize >= 16:
            strtab_off = sections[sh_link][1]
            count = sh_size // sh_entsize
            for i in range(count):
                sym_off = sh_offset + i * sh_entsize
                st_name, st_value, st_size, st_info, st_other, st_shndx = struct.unpack_from(
                    "<IIIBBH", data, sym_off
                )
                if st_name == 0:
                    continue
                end = data.find(b"\x00", strtab_off + st_name)
                name = data[strtab_off + st_name : end]
                if name == target_bytes:
                    return st_value

    raise RuntimeError(
        f"Symbol '{symbol_name}' not found in {elf_path}. Re-flash the updated firmware with 'cargo run' first."
    )


def swd_read_words(addr: int, count: int, speed: int = 400) -> list[int]:
    """Reads `count` 32-bit words from target memory at `addr` via probe-rs."""
    cmd = [
        "probe-rs",
        "read",
        "--chip",
        "nRF52840_xxAA",
        "--speed",
        str(speed),
        "b32",
        f"0x{addr:08X}",
        str(count),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        err = (res.stderr or res.stdout).strip()
        print(
            f"\nError communicating via XIAO DebugMate (SWD):\n{err}\n\n"
            "Tip: If 'cargo run' is still running in another terminal, press Ctrl+C there so DebugMate SWD is free.",
            file=sys.stderr,
        )
        sys.exit(1)

    words = []
    for line in res.stdout.splitlines():
        if ":" not in line:
            continue
        _, hex_part = line.split(":", 1)
        for token in hex_part.strip().split():
            words.append(int(token, 16))
    return words[:count]


def swd_write_words(addr: int, values: list[int], speed: int = 400):
    """Writes 32-bit words to target memory at `addr` via probe-rs."""
    cmd = [
        "probe-rs",
        "write",
        "--chip",
        "nRF52840_xxAA",
        "--speed",
        str(speed),
        "b32",
        f"0x{addr:08X}",
    ] + [f"0x{v:08X}" for v in values]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        err = (res.stderr or res.stdout).strip()
        print(f"\nError writing via XIAO DebugMate (SWD):\n{err}", file=sys.stderr)
        sys.exit(1)


def get_swd_mailbox_addr(args) -> int:
    elf_path = find_elf_path(getattr(args, "elf", None))
    try:
        return resolve_symbol_address(elf_path, "BBL_SWD_MAILBOX")
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


def open_serial(port: str, baud: int = 115200, timeout: float = 3.0):
    if not serial:
        print("Error: 'pyserial' is required for serial communication. Install via 'pip install pyserial'.", file=sys.stderr)
        sys.exit(1)
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except Exception as e:
        print(f"Error opening serial port {port}: {e}", file=sys.stderr)
        sys.exit(1)


def print_status_summary(transport: str, jedec: str, wr_ptr: int, sessions: int, used_bytes: int, capacity: int):
    pct = (used_bytes / capacity) * 100.0 if capacity > 0 else 0.0
    rec_sec = used_bytes / (50.0 * FRAME_SIZE)
    mins = int(rec_sec // 60)
    secs = int(rec_sec % 60)

    print("\n--- WingFC QSPI Blackbox Status ---")
    print(f"Transport      : {transport}")
    print(f"Flash Hardware : Puya P25Q16H (JEDEC: 0x{jedec})")
    print(f"Write Pointer  : 0x{wr_ptr:06X}")
    print(f"Flight Sessions: {sessions}")
    print(f"Used Capacity  : {used_bytes:,} / {capacity:,} bytes ({pct:.1f}%)")
    print(f"Logged Time    : {mins}m {secs:02d}s (approx. at 50 Hz)")


def cmd_info(args):
    if should_use_swd(args):
        mb_addr = get_swd_mailbox_addr(args)
        print(f"Connecting to WingFC via XIAO DebugMate SWD (Mailbox @ 0x{mb_addr:08X})...")
        words = swd_read_words(mb_addr, 8)
        magic, write_ptr, capacity, sessions, status, cmd, _, _ = words
        if magic != SWD_MAILBOX_MAGIC:
            print(
                f"Error: SWD Mailbox magic mismatch (got 0x{magic:08X}, expected 0x{SWD_MAILBOX_MAGIC:08X}).\n"
                "Make sure the latest firmware is flashed and running on the board.",
                file=sys.stderr,
            )
            sys.exit(1)
        used_bytes = max(0, write_ptr - LOG_START_ADDR)
        print_status_summary("XIAO DebugMate (SWD)", "856015", write_ptr, sessions, used_bytes, capacity)
        return

    port = args.port or find_wingfc_usb_port() or "/dev/ttyACM0"
    print(f"Connecting to WingFC on {port}...")
    ser = open_serial(port, args.baud)

    ser.write(b"BBL_INFO\r\n")
    line = ser.readline().decode("latin1", errors="replace").strip()
    ser.close()

    if not line.startswith("OK"):
        print(f"Unexpected response from flight controller: {line}", file=sys.stderr)
        sys.exit(1)

    kv = {}
    for part in line.split()[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            kv[k] = v

    jedec = kv.get("JEDEC", "856015")
    used_bytes = int(kv.get("USED_BYTES", 0))
    capacity = int(kv.get("CAPACITY", 2080768))
    wr_ptr = int(kv.get("WR_PTR", "0x004000"), 16)
    sessions = int(kv.get("SESSIONS", "1"))
    print_status_summary(f"USB CDC ({port})", jedec, wr_ptr, sessions, used_bytes, capacity)


def cmd_dump(args):
    if should_use_swd(args):
        mb_addr = get_swd_mailbox_addr(args)
        print(f"Initiating SWD binary dump via XIAO DebugMate (Mailbox @ 0x{mb_addr:08X}) to {args.output}...")
        words = swd_read_words(mb_addr, 8)
        magic, write_ptr, capacity, sessions, status, _, _, _ = words
        if magic != SWD_MAILBOX_MAGIC:
            print(f"Error: SWD Mailbox magic mismatch (0x{magic:08X}). Is the target running?", file=sys.stderr)
            sys.exit(1)

        size = max(0, write_ptr - LOG_START_ADDR)
        print(f"Streaming {size:,} bytes of raw blackbox logs over SWD...")
        received = bytearray()
        start_time = time.time()

        addr = LOG_START_ADDR
        while addr < write_ptr:
            chunk_len = min(SWD_BUF_BYTES, write_ptr - addr)
            # Write status=BUSY(1), cmd=READ_CHUNK(3), cmd_addr=addr, cmd_len=chunk_len at +0x10
            swd_write_words(mb_addr + 0x10, [SWD_STATUS_BUSY, SWD_CMD_READ_CHUNK, addr, chunk_len])

            # Wait for blackbox_task to complete the 4KB QSPI DMA read into RAM
            for _ in range(50):
                st_words = swd_read_words(mb_addr + 0x10, 2)
                st_status, st_cmd = st_words[0], st_words[1]
                if st_cmd == SWD_CMD_NONE:
                    break
                time.sleep(0.01)

            if st_status == SWD_STATUS_ERR_ARMED:
                print("\nERROR: Aircraft is currently armed! Dump is locked out for flight safety.", file=sys.stderr)
                sys.exit(1)
            if st_status != SWD_STATUS_IDLE:
                print(f"\nERROR: QSPI read failed with status {st_status} at 0x{addr:06X}.", file=sys.stderr)
                sys.exit(1)

            word_count = (chunk_len + 3) // 4
            buf_words = swd_read_words(mb_addr + 0x20, word_count)
            chunk_bytes = bytearray()
            for w in buf_words:
                chunk_bytes.extend(struct.pack("<I", w))
            received.extend(chunk_bytes[:chunk_len])

            addr += chunk_len
            pct = (len(received) / size) * 100.0 if size > 0 else 100.0
            sys.stdout.write(f"\rProgress: {len(received):,}/{size:,} bytes ({pct:.1f}%)")
            sys.stdout.flush()

        print()
        calc_crc = binascii.crc32(received) & 0xFFFFFFFF
        with open(args.output, "wb") as f:
            f.write(received)

        elapsed = time.time() - start_time
        rate_kb = (len(received) / 1024.0) / elapsed if elapsed > 0 else 0
        print(f"Dump complete (CRC-32: 0x{calc_crc:08X}): wrote {len(received):,} bytes in {elapsed:.2f}s ({rate_kb:.1f} KB/s)")
        return

    port = args.port or find_wingfc_usb_port() or "/dev/ttyACM0"
    print(f"Initiating binary dump from {port} to {args.output}...")
    ser = open_serial(port, args.baud, timeout=5.0)

    ser.write(b"BBL_DUMP\r\n")

    line = ser.readline().decode("latin1", errors="replace").strip()
    if "ERR:ARMED" in line:
        print("ERROR: Aircraft is currently armed! USB dump is locked out for flight safety.", file=sys.stderr)
        ser.close()
        sys.exit(1)

    if not line.startswith("START_DUMP"):
        print(f"Error starting dump: {line}", file=sys.stderr)
        ser.close()
        sys.exit(1)

    size = 0
    for part in line.split():
        if part.startswith("SIZE="):
            size = int(part.split("=")[1])

    print(f"Streaming {size:,} bytes of raw blackbox logs...")
    received = bytearray()
    start_time = time.time()

    while len(received) < size:
        to_read = min(4096, size - len(received))
        chunk = ser.read(to_read)
        if not chunk:
            print("\nError: Timed out waiting for dump data stream.", file=sys.stderr)
            break
        received.extend(chunk)
        pct = (len(received) / size) * 100.0 if size > 0 else 100.0
        sys.stdout.write(f"\rProgress: {len(received):,}/{size:,} bytes ({pct:.1f}%)")
        sys.stdout.flush()

    print()
    footer = ser.readline().decode("latin1", errors="replace").strip()
    ser.close()

    expected_crc = None
    if "END_DUMP" in footer and "CRC32=" in footer:
        for part in footer.split():
            if part.startswith("CRC32="):
                expected_crc = int(part.split("=")[1], 16)

    calc_crc = binascii.crc32(received) & 0xFFFFFFFF
    if expected_crc is not None:
        if calc_crc == expected_crc:
            print(f"CRC-32 Check: PASS (0x{calc_crc:08X})")
        else:
            print(f"CRC-32 Check: MISMATCH (Calculated: 0x{calc_crc:08X}, Expected: 0x{expected_crc:08X})", file=sys.stderr)

    with open(args.output, "wb") as f:
        f.write(received)

    elapsed = time.time() - start_time
    rate_kb = (len(received) / 1024.0) / elapsed if elapsed > 0 else 0
    print(f"Dump complete: wrote {len(received):,} bytes in {elapsed:.2f}s ({rate_kb:.1f} KB/s)")


def decode_frame(data: bytes):
    """Decodes a single 32-byte BlackboxFrame."""
    if len(data) != 32:
        return None
    if data[0] != MAGIC_BYTE or data[1] != VERSION_BYTE:
        return None
    if crc8_itu(data[:31]) != data[31]:
        return None

    status_flags = data[2]
    seq_lo = data[3]
    timestamp_ms = struct.unpack("<I", data[4:8])[0]
    gx, gy, gz = struct.unpack("<hhh", data[8:14])
    ax, ay, az = struct.unpack("<hhh", data[14:20])
    roll_mrad, pitch_mrad, yaw_mrad = struct.unpack("<hhh", data[20:26])

    # Unpack actuators (3x 10-bit in 32-bit uint)
    act_packed = struct.unpack("<I", data[26:30])[0]
    servo1 = (act_packed & 0x3FF) + 1000
    servo2 = ((act_packed >> 10) & 0x3FF) + 1000
    throttle = ((act_packed >> 20) & 0x3FF) + 1000

    rad_to_deg = 180.0 / math.pi
    return {
        "time_ms": timestamp_ms,
        "seq": seq_lo,
        "status": status_flags,
        "gx_rads": (gx / 1000.0),
        "gy_rads": (gy / 1000.0),
        "gz_rads": (gz / 1000.0),
        "ax_ms2": (ax / 100.0),
        "ay_ms2": (ay / 100.0),
        "az_ms2": (az / 100.0),
        "roll_deg": (roll_mrad / 1000.0) * rad_to_deg,
        "pitch_deg": (pitch_mrad / 1000.0) * rad_to_deg,
        "yaw_deg": (yaw_mrad / 1000.0) * rad_to_deg,
        "servo1_us": servo1,
        "servo2_us": servo2,
        "throttle_us": throttle,
    }


def cmd_export_csv(args):
    print(f"Decoding binary log {args.input} to CSV {args.output}...")
    with open(args.input, "rb") as f:
        raw = f.read()

    total_chunks = len(raw) // FRAME_SIZE
    valid_frames = 0
    skipped = 0

    fieldnames = [
        "time_ms", "seq", "status",
        "gx_rads", "gy_rads", "gz_rads",
        "ax_ms2", "ay_ms2", "az_ms2",
        "roll_deg", "pitch_deg", "yaw_deg",
        "servo1_us", "servo2_us", "throttle_us",
    ]

    with open(args.output, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for i in range(total_chunks):
            chunk = raw[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
            if chunk == b"\xFF" * FRAME_SIZE:
                skipped += 1
                continue
            frame = decode_frame(chunk)
            if frame:
                writer.writerow(frame)
                valid_frames += 1
            else:
                skipped += 1

    print(f"Export complete: {valid_frames:,} valid frames exported ({skipped:,} empty/invalid skipped)")


def cmd_erase(args):
    if should_use_swd(args):
        mb_addr = get_swd_mailbox_addr(args)
        if not args.yes:
            confirm = input("Are you sure you want to erase flight logs via XIAO DebugMate SWD? (y/N): ").strip().lower()
            if confirm != "y":
                print("Erase cancelled.")
                return

        words = swd_read_words(mb_addr, 8)
        if words[0] != SWD_MAILBOX_MAGIC:
            print(f"Error: SWD Mailbox magic mismatch (0x{words[0]:08X}).", file=sys.stderr)
            sys.exit(1)

        erase_cmd = SWD_CMD_ERASE_ALL if getattr(args, "full", False) else SWD_CMD_ERASE_USED
        print("Erasing blackbox flash sectors via XIAO DebugMate SWD...")
        swd_write_words(mb_addr + 0x10, [SWD_STATUS_BUSY, erase_cmd, 0, 0])

        while True:
            st_words = swd_read_words(mb_addr + 0x10, 4)
            st_status, st_cmd, done_sectors, total_sectors = st_words
            if st_status == SWD_STATUS_ERR_ARMED:
                print("\nERROR: Aircraft is armed! Flash erase blocked for flight safety.", file=sys.stderr)
                sys.exit(1)
            if total_sectors > 0:
                sys.stdout.write(f"\rErasing sectors: {done_sectors}/{total_sectors}...")
                sys.stdout.flush()
            if st_cmd == SWD_CMD_NONE and st_status == SWD_STATUS_IDLE:
                print("\nErase verified: Log sectors cleared and write pointer reset to 0x004000.")
                break
            time.sleep(0.05)
        return

    port = args.port or find_wingfc_usb_port() or "/dev/ttyACM0"
    if not args.yes:
        confirm = input(f"Are you sure you want to erase all flight logs on {port}? (y/N): ").strip().lower()
        if confirm != "y":
            print("Erase cancelled.")
            return

    print(f"Connecting to WingFC on {port}...")
    ser = open_serial(port, args.baud, timeout=10.0)

    ser.write(b"BBL_ERASE\r\n")

    while True:
        line = ser.readline().decode("latin1", errors="replace").strip()
        if not line:
            continue
        if "ERR:ARMED" in line:
            print("ERROR: Aircraft is armed! Flash erase blocked for flight safety.", file=sys.stderr)
            break
        if "ERASING" in line:
            sys.stdout.write(f"\r{line}...")
            sys.stdout.flush()
        elif "OK ERASED" in line:
            print("\nErase verified: All 508 log sectors cleared, write pointer reset.")
            break
        elif "ERR" in line:
            print(f"\nErase failed: {line}", file=sys.stderr)
            break

    ser.close()


def main():
    parser = argparse.ArgumentParser(description="WingFC Blackbox Log Offload & Analysis Tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Info
    p_info = subparsers.add_parser("info", help="Inspect flash usage and session info")
    p_info.add_argument("--port", "-p", help="USB serial port (auto-detects USB or DebugMate SWD if omitted)")
    p_info.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    p_info.add_argument("--swd", action="store_true", help="Force XIAO DebugMate SWD (probe-rs) transport")
    p_info.add_argument("--elf", help="Optional path to wingfc-firmware ELF for SWD symbol lookup")

    # Dump
    p_dump = subparsers.add_parser("dump", help="Download raw binary log stream")
    p_dump.add_argument("output", help="Output file path (e.g. flight.bbl)")
    p_dump.add_argument("--port", "-p", help="USB serial port (auto-detects USB or DebugMate SWD if omitted)")
    p_dump.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    p_dump.add_argument("--swd", action="store_true", help="Force XIAO DebugMate SWD (probe-rs) transport")
    p_dump.add_argument("--elf", help="Optional path to wingfc-firmware ELF for SWD symbol lookup")

    # Export CSV
    p_csv = subparsers.add_parser("export-csv", help="Decode binary .bbl log into CSV format")
    p_csv.add_argument("input", help="Input binary log path (.bbl)")
    p_csv.add_argument("output", help="Output CSV file path (.csv)")

    # Erase
    p_erase = subparsers.add_parser("erase", help="Erase log storage before flight")
    p_erase.add_argument("--port", "-p", help="USB serial port (auto-detects USB or DebugMate SWD if omitted)")
    p_erase.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    p_erase.add_argument("--swd", action="store_true", help="Force XIAO DebugMate SWD (probe-rs) transport")
    p_erase.add_argument("--full", action="store_true", help="Erase all 508 sectors (2 MB) instead of used sectors")
    p_erase.add_argument("--elf", help="Optional path to wingfc-firmware ELF for SWD symbol lookup")
    p_erase.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompt")

    args = parser.parse_args()

    if args.command == "info":
        cmd_info(args)
    elif args.command == "dump":
        cmd_dump(args)
    elif args.command == "export-csv":
        cmd_export_csv(args)
    elif args.command == "erase":
        cmd_erase(args)


if __name__ == "__main__":
    main()
