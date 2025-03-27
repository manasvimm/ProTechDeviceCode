import asyncio
import csv
import time
from bleak import BleakClient

BLE_DEVICE_ADDRESS = "dc:54:75:eb:2b:09"  # Replace with your ESP32's MAC Address
CHARACTERISTIC_UUID = "abcdef01-1234-5678-1234-56789abcdef0"

# Handle received BLE data
async def notification_handler(sender, data):
    decoded_data = data.decode("utf-8")
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")  # Get current date & time
    print(f"{timestamp} - Received: {decoded_data}")

    # Save to CSV
    with open("ble_data.csv", "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp] + decoded_data.split(","))  # Add timestamp

async def main():
    async with BleakClient(BLE_DEVICE_ADDRESS) as client:
        print("Connected to ESP32!")

        # Enable notifications
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        try:
            while True:  # Infinite loop to keep script running
                await asyncio.sleep(1)  # Keeps the event loop alive
        except KeyboardInterrupt:
            print("Logging stopped manually.")

        # Stop notifications when exiting
        await client.stop_notify(CHARACTERISTIC_UUID)

asyncio.run(main())
