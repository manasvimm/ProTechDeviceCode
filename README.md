# DanmarDeviceCode
Contains device codes for both FFT utilizing an Adafruit ESP32 Feather S3 and transmitting data from an Adafruit BNO055 with BLE connections to a python controlled connection.
The file BluetoothTest.io codes for collecting data from the ESP32 and IMU, this in combiniation with running the ble_logger2.py file causes for a local file to be created and logg time stamps and acceleration in a csv file.
The file ESP32_FFT.io handles data collection and analysis for the device. This code collects data from the BNO055, performs a FFT on the data, and then runs a motion detection algorithm
