# Test Version
I changed some parameters to make the reading more precise by also inserting a calibration factor of verse based on the speeds to set it correctly in each phase





# esp32_treadmill_ble
Send treadmill data without Bluetooth to Garmin devices and other

This code for Esp32 allows you to send data from a positioned IR sensor to Garmin, Zwift, and other devices that support Bluetooth sensor connectivity.

If you have a treadmill that doesn't have Bluetooth and want to send your workout speed to third-party apps like Zwift or even to your Garmin watch (if it supports Bluetooth), this code is what you're looking for.

This code is designed to work with an ESP32 connected to an IR sensor like this: https://www.amazon.it/AZDelivery-Infrarossi-distanza-rilevamento-Raspberry/dp/B089QJTVB1?th=1

To make the sensor read the speed of the treadmill, you'll need to make one or more white marks on the treadmill and adjust the code accordingly by measuring the distance between the marks.

The sensor must be connected:
VCC -> 3.3V
GND -> GND
OUT -> GPIO 15

The code is fully commented for customization.
