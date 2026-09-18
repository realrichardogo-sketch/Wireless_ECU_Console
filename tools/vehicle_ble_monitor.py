#!/usr/bin/env python3
"""Wireless ECU Console BLE NUS frame monitor.

Usage:
    python vehicle_ble_monitor.py
    python vehicle_ble_monitor.py --name "Wireless ECU Console"
"""

from __future__ import annotations

import argparse
import asyncio
import struct


SOF = b"\x55\xAA"
FRAME_SIZE = 15
VERSION = 0x01
TYPE_VEHICLE_STATUS = 0x10
PAYLOAD_SIZE = 6

DEFAULT_DEVICE_NAME = "Wireless ECU Console"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

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


async def monitor(device_name: str, timeout: float) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("bleak is missing. Install it with: python -m pip install bleak")
        return 2

    print(f"Scanning for {device_name!r} ...")
    device = await BleakScanner.find_device_by_filter(
        lambda candidate, advertisement: (
            advertisement.local_name == device_name
            or candidate.name == device_name
        ),
        timeout=timeout,
    )

    if device is None:
        print(f"Device {device_name!r} was not found.")
        print("Confirm that RTT shows 'BLE NUS advertising' and try again.")
        return 1

    receive_buffer = bytearray()

    def notification_handler(_sender, data: bytearray) -> None:
        receive_buffer.extend(data)
        for frame in extract_frames(receive_buffer):
            print(decode_frame(frame), flush=True)

    print(f"Connecting to {device.name or device.address} ...")
    async with BleakClient(device) as client:
        print("Connected; subscribing to NUS TX notifications.")
        await client.start_notify(NUS_TX_UUID, notification_handler)
        print("Subscribed. Press Ctrl+C to stop.\n")

        while client.is_connected:
            await asyncio.sleep(1.0)

    print("BLE connection closed.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive Wireless ECU Console frames over BLE NUS."
    )
    parser.add_argument("--name", default=DEFAULT_DEVICE_NAME)
    parser.add_argument("--scan-timeout", type=float, default=15.0)
    args = parser.parse_args()

    try:
        return asyncio.run(monitor(args.name, args.scan_timeout))
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
