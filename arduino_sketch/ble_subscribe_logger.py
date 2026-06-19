#!/usr/bin/env python3
"""
ENGG499 LAB 9 BLE OUTPUT COLLECTOR
Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu

Note: Recognizing that this is largely unnecessary for this lab but helpful
for my own personal convenience, this was initially thrown together via StackOverflow + 
GenAI code + my own UUIDs/etc. It is NOT something that I've machined/documented extensively,
it's merely a quick helper.

This requires Bleak:
    pip install bleak
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "Missing dependency: bleak\n"
        "Install it with: pip install bleak"
    ) from exc


TARGET_NAMES = (
    "ENGG499_BOARD",
)

BOARD_SERVICE_UUID = "454E4747-3439-0000-8000-0F4F0C499009"
LOG_CHARACTERISTIC_UUID = "4C4F4701-8F16-4A52-A3A2-0F4F0C499009"
AFIB_CHARACTERISTIC_UUID = "41464942-8F16-4A52-A3A2-0F4F0C499109"
HEART_RATE_CHARACTERISTIC_UUID = "48520001-8F16-4A52-A3A2-0F4F0C499009"


@dataclass(frozen=True)
class CharacteristicSpec:
    name: str
    uuid: str
    decoder: Callable[[bytes], str]


def decode_text(raw: bytes) -> str:
    return raw.rstrip(b"\x00").decode("utf-8", errors="replace")


def decode_heart_rate(raw: bytes) -> str:
    if len(raw) < 2:
        return "<invalid uint16>"
    return str(int.from_bytes(raw[:2], byteorder="little", signed=False))


CHARACTERISTICS = (
    CharacteristicSpec("log_output", LOG_CHARACTERISTIC_UUID, decode_text),
    CharacteristicSpec("afib_prediction", AFIB_CHARACTERISTIC_UUID, decode_text),
    CharacteristicSpec("heart_rate_bpm", HEART_RATE_CHARACTERISTIC_UUID, decode_heart_rate),
)


class CsvNotificationLogger:
    def __init__(self, output_path: Path) -> None:
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ["timestamp_iso", "source", "characteristic", "uuid", "decoded_value"]
        )
        self._file.flush()

    def write(self, source: str, spec: CharacteristicSpec, raw: bytes) -> None:
        timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        decoded_value = spec.decoder(raw)
        self._writer.writerow([timestamp, source, spec.name, spec.uuid, decoded_value])
        self._file.flush()
        print(f"{timestamp} [{source}] {spec.name}: {decoded_value}")

    def close(self) -> None:
        self._file.close()


def default_output_path() -> Path:
    script_dir = Path(__file__).resolve().parent
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return script_dir / "logs" / f"ble_notifications_{timestamp}.csv"


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Connect to the ENGG499 Lab 9 board and subscribe to the log, AFIB, "
            "and heart-rate characteristics."
        )
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=15.0,
        help="BLE scan timeout in seconds before giving up. Default: 15",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output_path(),
        help="CSV file to write notifications to. Default: logs/<timestamp>.csv beside this script",
    )
    return parser


def normalize_name(name: str | None) -> str:
    return (name or "").strip().casefold()


async def find_target_device(scan_timeout: float):
    accepted_names = {normalize_name(name) for name in TARGET_NAMES}
    target_service_uuid = BOARD_SERVICE_UUID.casefold()

    def matches(device, advertisement_data) -> bool:
        advertised_name = normalize_name(getattr(advertisement_data, "local_name", None))
        device_name = normalize_name(getattr(device, "name", None))
        service_uuids = {
            str(uuid).casefold()
            for uuid in getattr(advertisement_data, "service_uuids", []) or []
        }
        return (
            target_service_uuid in service_uuids
            or advertised_name in accepted_names
            or device_name in accepted_names
        )

    print(
        "Scanning for BLE board service "
        + BOARD_SERVICE_UUID
        + " or device names: "
        + ", ".join(TARGET_NAMES)
        + f" (timeout: {scan_timeout:.1f}s)"
    )
    device = await BleakScanner.find_device_by_filter(matches, timeout=scan_timeout)
    if device is None:
        raise RuntimeError(
            "Could not find the ENGG499 Lab 9 board. Confirm the board is powered, "
            "advertising, and within range."
        )

    print(f"Found device: name={device.name!r}, address={device.address}")
    return device


def make_notification_handler(
    logger: CsvNotificationLogger, spec: CharacteristicSpec
) -> Callable[[object, bytearray], None]:
    def handle_notification(_sender: object, data: bytearray) -> None:
        logger.write("notify", spec, bytes(data))

    return handle_notification


async def describe_characteristics(client: BleakClient) -> None:
    services = client.services
    if services is None:
        print("Service metadata unavailable; continuing with direct UUID access.")
        return

    for spec in CHARACTERISTICS:
        characteristic = services.get_characteristic(spec.uuid)
        if characteristic is None:
            print(f"Warning: {spec.name} [{spec.uuid}] was not present in discovered metadata.")
            continue
        properties = ", ".join(characteristic.properties)
        print(f"Characteristic ready: {spec.name} [{spec.uuid}] properties={properties}")


async def monitor_device(scan_timeout: float, output_path: Path) -> None:
    logger = CsvNotificationLogger(output_path)
    disconnect_event = asyncio.Event()

    def handle_disconnect(_: BleakClient) -> None:
        print("Device disconnected.")
        disconnect_event.set()

    device = await find_target_device(scan_timeout)

    try:
        print("Connecting...")
        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            await describe_characteristics(client)

            for spec in CHARACTERISTICS:
                print(f"Subscribing to {spec.name} [{spec.uuid}]")
                await client.start_notify(
                    spec.uuid, make_notification_handler(logger, spec)
                )

            for spec in CHARACTERISTICS:
                raw_value = bytes(await client.read_gatt_char(spec.uuid))
                logger.write("read", spec, raw_value)

            print(f"Logging notifications to: {output_path}")
            print("Listening for updates. Press Ctrl+C to stop.")

            while client.is_connected and not disconnect_event.is_set():
                await asyncio.sleep(0.5)

    finally:
        logger.close()


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    try:
        asyncio.run(monitor_device(args.scan_timeout, args.output))
    except KeyboardInterrupt:
        print("\nStopping BLE logger.")
        return 0
    except (BleakError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
