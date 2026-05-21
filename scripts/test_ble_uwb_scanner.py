#!/usr/bin/env python3
"""
BLE Scanner for UWB Tags
Listens for advertisements from UWB tags with name "UWB-T<id>"
and parses manufacturer data containing position information.
"""

import asyncio
import struct
from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData


def parse_uwb_manufacturer_data(data: bytes) -> dict:
    """
    Parse UWB tag manufacturer data.

    Format (7 bytes):
      [0..1] Company ID 0xFFFF (little-endian)
      [2]    TAG_ID
      [3..4] x position in cm (int16, little-endian)
      [5..6] y position in cm (int16, little-endian)
    """
    if len(data) < 7:
        return None

    try:
        company_id = struct.unpack('<H', data[0:2])[0]
        tag_id = data[2]
        x_pos = struct.unpack('<h', data[3:5])[0]  # signed int16
        y_pos = struct.unpack('<h', data[5:7])[0]  # signed int16

        return {
            'company_id': company_id,
            'tag_id': tag_id,
            'x_cm': x_pos,
            'y_cm': y_pos
        }
    except Exception as e:
        print(f"Error parsing manufacturer data: {e}")
        return None


def detection_callback(device: BLEDevice, advertisement_data: AdvertisementData):
    """Callback for when a BLE device is detected."""

    # Check if device name matches UWB tag pattern
    if device.name and device.name.startswith("UWB-T"):
        print(f"\n{'='*60}")
        print(f"UWB Tag Detected: {device.name}")
        print(f"Address: {device.address}")
        print(f"RSSI: {advertisement_data.rssi} dBm")

        # Parse manufacturer data
        if advertisement_data.manufacturer_data:
            # Manufacturer data is a dict with company_id as key
            for company_id, data in advertisement_data.manufacturer_data.items():
                print(f"Manufacturer Data (Company ID: 0x{company_id:04X}):")
                print(f"  Raw bytes: {data.hex()}")

                # Parse position data
                parsed = parse_uwb_manufacturer_data(bytes([company_id & 0xFF, (company_id >> 8) & 0xFF]) + data)
                if parsed:
                    print(f"  Tag ID: {parsed['tag_id']}")
                    print(f"  Position: X={parsed['x_cm']} cm, Y={parsed['y_cm']} cm")
        else:
            print("No manufacturer data")

        # Show all advertisement data for debugging
        if advertisement_data.service_data:
            print(f"Service Data: {advertisement_data.service_data}")
        if advertisement_data.service_uuids:
            print(f"Service UUIDs: {advertisement_data.service_uuids}")

        print(f"{'='*60}\n")


async def scan_for_uwb_tags(duration: float = 30.0):
    """
    Scan for UWB tag BLE advertisements.

    Args:
        duration: Scan duration in seconds (default 30s)
    """
    print(f"Starting BLE scan for UWB tags (duration: {duration}s)...")
    print("Looking for devices with name pattern 'UWB-T*'")
    print(f"{'='*60}\n")

    scanner = BleakScanner(detection_callback=detection_callback)

    await scanner.start()
    await asyncio.sleep(duration)
    await scanner.stop()

    print(f"\n{'='*60}")
    print("Scan completed")
    print(f"{'='*60}")


def main():
    """Main entry point."""
    try:
        # Scan for 30 seconds (can be adjusted)
        asyncio.run(scan_for_uwb_tags(duration=30.0))
    except KeyboardInterrupt:
        print("\nScan interrupted by user")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
