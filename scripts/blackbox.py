#!/usr/bin/env python3
"""
WingFC Blackbox Log Offload & Analysis Tool.

Communicates with WingFC-rs flight controller over USB CDC-ACM to inspect,
download, and decode high-rate 50 Hz binary flight logs from onboard QSPI NOR flash.
"""

import argparse
import binascii
import csv
import math
import struct
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


def find_default_port() -> str:
    """Auto-detects USB serial port if none provided."""
    if not serial:
        return "/dev/ttyACM0"
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        if "wingfc" in desc or "wingfc" in mfg or "seeed" in desc or "xiao" in desc:
            return p.device
    for p in ports:
        if "ttyacm" in p.device.lower() or "usbmodem" in p.device.lower():
            return p.device
    return "/dev/ttyACM0"


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


def cmd_info(args):
    port = args.port or find_default_port()
    print(f"Connecting to WingFC on {port}...")
    ser = open_serial(port, args.baud)

    ser.write(b"BBL_INFO\r\n")
    line = ser.readline().decode("latin1", errors="replace").strip()
    ser.close()

    if not line.startswith("OK"):
        print(f"Unexpected response from flight controller: {line}", file=sys.stderr)
        sys.exit(1)

    # Parse key-values
    kv = {}
    for part in line.split()[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            kv[k] = v

    jedec = kv.get("JEDEC", "Unknown")
    used_bytes = int(kv.get("USED_BYTES", 0))
    capacity = int(kv.get("CAPACITY", 2080768))
    wr_ptr = kv.get("WR_PTR", "0x004000")
    sessions = kv.get("SESSIONS", "1")

    pct = (used_bytes / capacity) * 100.0 if capacity > 0 else 0.0
    rec_sec = used_bytes / (50.0 * FRAME_SIZE)
    mins = int(rec_sec // 60)
    secs = int(rec_sec % 60)

    print("\n--- WingFC QSPI Blackbox Status ---")
    print(f"Flash Hardware : Puya P25Q16H (JEDEC: 0x{jedec})")
    print(f"Write Pointer  : {wr_ptr}")
    print(f"Flight Sessions: {sessions}")
    print(f"Used Capacity  : {used_bytes:,} / {capacity:,} bytes ({pct:.1f}%)")
    print(f"Logged Time    : {mins}m {secs:02d}s (approx. at 50 Hz)")


def cmd_dump(args):
    port = args.port or find_default_port()
    print(f"Initiating binary dump from {port} to {args.output}...")
    ser = open_serial(port, args.baud, timeout=5.0)

    ser.write(b"BBL_DUMP\r\n")

    # Read first line
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
    port = args.port or find_default_port()
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
    p_info.add_argument("--port", "-p", help="Serial port (auto-detect if omitted)")
    p_info.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")

    # Dump
    p_dump = subparsers.add_parser("dump", help="Download raw binary log stream")
    p_dump.add_argument("output", help="Output file path (e.g. flight.bbl)")
    p_dump.add_argument("--port", "-p", help="Serial port (auto-detect if omitted)")
    p_dump.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")

    # Export CSV
    p_csv = subparsers.add_parser("export-csv", help="Decode binary .bbl log into CSV format")
    p_csv.add_argument("input", help="Input binary log path (.bbl)")
    p_csv.add_argument("output", help="Output CSV file path (.csv)")

    # Erase
    p_erase = subparsers.add_parser("erase", help="Erase log storage before flight")
    p_erase.add_argument("--port", "-p", help="Serial port (auto-detect if omitted)")
    p_erase.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
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
