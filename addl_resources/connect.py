# EcoDrone: Autonomous Environmental Monitoring
# Simple python script to connect to the drone's ESP32's BluetoothLE signal to retrieve sensor data
# Author: Brandon Lee, brandon.kf.lee@gmail.com
# Code partially derived from hbldh's service_explorer.py example code (https://github.com/hbldh/bleak/blob/develop/examples/service_explorer.py)

import platform
import asyncio
from bleak import BleakClient, BleakScanner

drone_name = "EcoDrone"
drone_transmit_uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e" # WARNING: this UUID is hard linked to the drone! 
macos_use_bdaddr = False # When true use Bluetooth address instead of UUID on macOS

async def main():
    if platform.system() == "Darwin":
        macos_use_bdaddr = True

    print("Starting scan...", end=" ", flush=True)

    device = await BleakScanner.find_device_by_name(
        drone_name, cb=dict(use_bdaddr=macos_use_bdaddr)
    )
    if device is None:
        print("Could not find device with name '%s'", drone_name)
        return

    print("Connecting to device...", end=" ", flush=True)

    async with BleakClient(device) as client:
        print("Connected")

        # Search through all BLE services and all their characteristics
        for service in client.services:
            for char in service.characteristics:

                if char.uuid == drone_transmit_uuid:
                    if "read" in char.properties:
                        try:
                            # If this characteristic has values to read, read it and write it into "value"
                            value = await client.read_gatt_char(char.uuid)
                        except Exception as e:
                            value = "Error: {e}"

                    if not value:
                        print("No value")
                    else:
                        print(value.decode("utf-8"))

        print("Disconnecting...", end=" ", flush=True)
    print("Disconnected")

asyncio.run(main())