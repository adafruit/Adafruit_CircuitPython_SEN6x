# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Simpletest for the SEN6x Driver
# The sensor model is detected automatically, or uncomment the init for your sensor
import time

import board

import adafruit_sen6x

# Initialize I2C
i2c = board.I2C()

# Initialize the sensor:
sensor = adafruit_sen6x.create_sensor(i2c)  # Detects the connected model
# sensor = adafruit_sen6x.SEN66(i2c)  # SEN66 sensors
# sensor = adafruit_sen6x.SEN62(i2c)  # SEN62 sensors
# sensor = adafruit_sen6x.SEN63C(i2c)  # SEN63C sensors
# sensor = adafruit_sen6x.SEN65(i2c)  # SEN65 sensors
# sensor = adafruit_sen6x.SEN68(i2c)  # SEN68 sensors
# sensor = adafruit_sen6x.SEN69C(i2c)  # SEN69C sensors
# sensor = adafruit_sen6x.SEN60(i2c)  # SEN60 sensors (discontinued)

# Read sensor info
print(f"Model: {type(sensor).__name__}")
print(f"Serial: {sensor.serial_number}")

# Check device status
status = sensor.device_status
print(f"Device {status}")

# Optional: Configure sensor before starting
# sensor.temperature_offset(offset=-2.0, slot=0)  # Apply -2°C offset

# VOC/NOx configuration examples (SEN65, SEN66, SEN68, SEN69C):
# sensor.voc_algorithm_tuning(index_offset=100)   # Adjust VOC baseline
# print(sensor.voc_algorithm) # Print VOC baseline

# CO2 configuration examples (SEN63C, SEN66, SEN69C):
# sensor.co2_automatic_self_calibration = False  # Disable ASC for lab testing
# sensor.ambient_pressure = 1020  # Set pressure in hPa
# sensor.sensor_altitude = 500    # Or set altitude in meters

# Start measurements
sensor.start_measurement()

# Wait for first measurement to be ready
print("Waiting for first measurement...")
time.sleep(2)
print("-" * 40)

# Units for each measurement, only those supported by your sensor are shown
labels = {
    "temperature": ("Temperature", "°C"),
    "humidity": ("Humidity", "%"),
    "pm1_0": ("PM1.0", "µg/m³"),
    "pm2_5": ("PM2.5", "µg/m³"),
    "pm4_0": ("PM4.0", "µg/m³"),
    "pm10": ("PM10", "µg/m³"),
    "voc_index": ("VOC Index", ""),
    "nox_index": ("NOx Index", ""),
    "hcho": ("HCHO", "ppb"),
    "co2": ("CO2", "ppm"),
    "nc_pm2_5": ("PM2.5 number", "#/cm³"),  # SEN60 only
}

# Read data continuously
while True:
    if sensor.data_ready:
        # Check for errors before reading
        sensor.check_sensor_errors()

        # Read all measurements
        data = sensor.all_measurements()

        # Display values (None = sensor still initializing)
        for key, (name, unit) in labels.items():
            if key not in data:
                continue
            value = data[key]
            if value is None:
                print(f"{name}: initializing...")
            else:
                print(f"{name}: {value:.1f} {unit}")
        print("-" * 40)
    time.sleep(2)
