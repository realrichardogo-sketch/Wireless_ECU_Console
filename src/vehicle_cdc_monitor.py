#!/usr/bin/env python3
"""Wireless ECU Console USB CDC frame monitor.

Usage:
    python vehicle_cdc_monitor.py COM7
"""

from __future__ import annotations

import argparse
import struct
import time


SOF = b"\x55\xAA"
FRAME_SIZE = 15
VERSION = 0x01
TYPE_VEHICLE_STATUS = 0x10
PAYLOAD_SIZE = 6

STATE_NAMES = {
    0: "NORMAL",
    1: "HIGH_RPM",
    2: "OVER_TEMP",
    3: "CRITICAL",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_frame(frame: bytes) -> str:
    if len(frame) != FRAME_SIZE:
        raise ValueError("invalid frame length")

    version = frame[2]
    message_type = frame[3]
    sequence = int.from_bytes(frame[4:6], "little")
    payload_length = frame[6]
    rpm = int.from_bytes(frame[7:9], "little")
    temperature = struct.unpack_from("<h", frame, 9)[0]
    state = frame[11]
    flags = frame[12]
    received_crc = int.from_bytes(frame[13:15], "little")
    calculated_crc = crc16_ccitt_false(frame[2:13])

    header_ok = (
        version == VERSION
        and message_type == TYPE_VEHICLE_STATUS
        and payload_length == PAYLOAD_SIZE
    )
    crc_ok = received_crc == calculated_crc
    state_name = STATE_NAMES.get(state, f"UNKNOWN({state})")
    fault = bool(flags & 0x01)

    return (
        f"SEQ:{sequence:5d} | RPM:{rpm:4d} | TEMP:{temperature:4d} C | "
        f"FAULT:{int(fault)} | STATE:{state_name:<9} | "
        f"HEADER:{'OK' if header_ok else 'BAD'} | "
        f"CRC:{'OK' if crc_ok else 'BAD'} | "
        f"RAW:{frame.hex(' ').upper()}"
    )


def extract_frames(buffer: bytearray):
    while True:
        sof_index = buffer.find(SOF)
        if sof_index < 0:
            if buffer[-1:] == SOF[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            return

        if sof_index > 0:
            del buffer[:sof_index]

        if len(buffer) < FRAME_SIZE:
            return

        candidate = bytes(buffer[:FRAME_SIZE])

        if candidate[2] != VERSION or candidate[6] != PAYLOAD_SIZE:
            del buffer[0]
            continue

        del buffer[:FRAME_SIZE]
        yield candidate


def main() -> int:
    try:
        import serial
    except ImportError:
        print("pyserial is missing. Install it with: python -m pip install pyserial")
        return 2

    parser = argparse.ArgumentParser(
        description="Receive Wireless ECU Console binary frames over USB CDC."
    )
    parser.add_argument("port", help="Windows COM port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    receive_buffer = bytearray()

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        ) as connection:
            connection.dtr = True
            connection.reset_input_buffer()
            print(f"Connected to {args.port} at {args.baud} 8N1; DTR=1")
            print("Press Ctrl+C to stop.\n")

            while True:
                chunk = connection.read(64)
                if chunk:
                    receive_buffer.extend(chunk)
                    for frame in extract_frames(receive_buffer):
                        print(decode_frame(frame), flush=True)
                else:
                    time.sleep(0.01)

    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
