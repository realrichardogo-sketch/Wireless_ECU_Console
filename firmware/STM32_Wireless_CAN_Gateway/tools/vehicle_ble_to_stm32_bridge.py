#!/usr/bin/env python3
"""Forward validated Wireless ECU Console NUS frames to STM32 Node A."""

import argparse
import asyncio
import sys
from dataclasses import dataclass

import serial
from bleak import BleakClient, BleakScanner

NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
FRAME_SIZE = 15
HEADER = b"\x55\xAA"

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


@dataclass(frozen=True)
class VehicleFrame:
    raw: bytes
    sequence: int
    rpm: int
    temperature_c: int
    fault: int
    state: int


def decode_frame(raw: bytes) -> VehicleFrame | None:
    if len(raw) != FRAME_SIZE:
        return None
    if raw[:2] != HEADER or raw[2] != 0x01 or raw[3] != 0x10:
        return None
    if raw[6] != 0x06 or raw[11] not in STATE_NAMES:
        return None

    received_crc = int.from_bytes(raw[13:15], "little")
    calculated_crc = crc16_ccitt_false(raw[2:13])
    if received_crc != calculated_crc:
        return None

    return VehicleFrame(
        raw=raw,
        sequence=int.from_bytes(raw[4:6], "little"),
        rpm=int.from_bytes(raw[7:9], "little"),
        temperature_c=int.from_bytes(raw[9:11], "little", signed=True),
        fault=1 if (raw[12] & 0x01) else 0,
        state=raw[11],
    )


class Bridge:
    def __init__(self, uart: serial.Serial) -> None:
        self.uart = uart
        self.buffer = bytearray()
        self.good_frames = 0
        self.bad_frames = 0
        self.lost_frames = 0
        self.last_sequence: int | None = None

    def notification(self, _sender, chunk: bytearray) -> None:
        self.buffer.extend(chunk)
        self._consume()

    def _consume(self) -> None:
        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                if self.buffer.endswith(b"\x55"):
                    self.buffer[:] = b"\x55"
                else:
                    self.buffer.clear()
                return

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < FRAME_SIZE:
                return

            raw = bytes(self.buffer[:FRAME_SIZE])
            frame = decode_frame(raw)
            if frame is None:
                self.bad_frames += 1
                del self.buffer[0]
                print(f"BLE RX invalid frame, bad={self.bad_frames}")
                continue

            del self.buffer[:FRAME_SIZE]
            self._track_sequence(frame.sequence)
            self.uart.write(frame.raw)
            self.uart.flush()
            self.good_frames += 1
            print(
                f"BLE RX SEQ:{frame.sequence:5d} RPM:{frame.rpm:4d} "
                f"TEMP:{frame.temperature_c:4d} C FAULT:{frame.fault} "
                f"STATE:{STATE_NAMES[frame.state]:9s} CRC:OK -> "
                f"UART TX {self.uart.port} {len(frame.raw)} bytes"
            )

    def _track_sequence(self, sequence: int) -> None:
        if self.last_sequence is not None:
            delta = (sequence - self.last_sequence) & 0xFFFF
            if 1 < delta < 0x8000:
                self.lost_frames += delta - 1
                print(f"WARNING: sequence gap {self.last_sequence} -> {sequence}")
        self.last_sequence = sequence


async def find_console(name: str, timeout: float):
    print(f"Scanning for '{name}' ...")
    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
    for device, advertisement in devices.values():
        if device.name == name or advertisement.local_name == name:
            return device
    return None


async def run(args: argparse.Namespace) -> None:
    with serial.Serial(args.port, args.baud, timeout=0, write_timeout=1) as uart:
        print(f"UART ready: {uart.port} @ {uart.baudrate} 8N1")
        device = await find_console(args.name, args.scan_timeout)
        if device is None:
            raise RuntimeError(f"BLE device '{args.name}' was not found")

        print(f"Connecting to {device.name} ({device.address}) ...")
        async with BleakClient(device) as client:
            bridge = Bridge(uart)
            await client.start_notify(NUS_TX_UUID, bridge.notification)
            print("Connected and subscribed; forwarding valid frames. Ctrl+C stops.")
            try:
                while client.is_connected:
                    await asyncio.sleep(1.0)
            finally:
                if client.is_connected:
                    await client.stop_notify(NUS_TX_UUID)
                print(
                    f"Stopped: good={bridge.good_frames} bad={bridge.bad_frames} "
                    f"lost={bridge.lost_frames}"
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BLE NUS to STM32 USART2 vehicle-frame bridge"
    )
    parser.add_argument("--port", required=True, help="USB-TTL port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--name", default="Wireless ECU Console")
    parser.add_argument("--scan-timeout", type=float, default=12.0)
    return parser.parse_args()


if __name__ == "__main__":
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("Stopped by user.")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
