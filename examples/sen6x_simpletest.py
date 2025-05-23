# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Example usage:
import time
import board
import adafruit_sen6x

# Initialize I2C
i2c = board.I2C()

# Create SEN66 instance
sensor = adafruit_sen6x.SEN66(i2c)

# Read sensor info
print(f"Product: {sensor.product_name}")
print(f"Serial: {sensor.serial_number}")

# Check device status
status = sensor.device_status
print(f"Device {status}")

# Optional: Configure sensor before starting
# sensor.set_temperature_offset(offset=-2.0, slot=0)  # Apply -2°C offset
# sensor.set_voc_algorithm_tuning(index_offset=100)   # Adjust VOC baseline

# CO2 configuration examples:
# sensor.co2_automatic_self_calibration = False  # Disable ASC for greenhouses
# sensor.ambient_pressure = 1020  # Set pressure in hPa
# sensor.sensor_altitude = 500    # Or set altitude in meters

# Start measurements
sensor.start_measurement()

# Wait for first measurement to be ready
print("Waiting for first measurement...")
time.sleep(2)

# Optional: Save VOC state periodically for power loss recovery
voc_state = None
state_save_time = time.monotonic()

# Read data continuously
while True:
    if sensor.data_ready:
        try:
            # Check for errors before reading
            sensor.check_sensor_errors()
            
            # Read all measurements
            data = sensor.read_measurement()
            
            # Display values (None = sensor still initializing)
            print(f"Temperature: {data['temperature']:.1f}°C" if data['temperature'] else "Temperature: initializing...")
            print(f"Humidity: {data['humidity']:.1f}%" if data['humidity'] else "Humidity: initializing...")
            print(f"PM2.5: {data['pm2_5']:.1f} µg/m³" if data['pm2_5'] else "PM2.5: initializing...")
            print(f"VOC Index: {data['voc_index']:.1f}" if data['voc_index'] else "VOC Index: initializing...")
            print(f"NOx Index: {data['nox_index']:.1f}" if data['nox_index'] else "NOx Index: initializing...")
            print(f"CO2: {data['co2']} ppm" if data['co2'] else "CO2: initializing...")
            
            # Save VOC state every 60 seconds
            if time.monotonic() - state_save_time > 10:
                voc_state = sensor.voc_algorithm_state
                state_save_time = time.monotonic()
                print("VOC state saved")
            
            # Optional: Read number concentrations
            # nc_data = sensor.read_number_concentration()
            # print(f"PM0.5 count: {nc_data['nc_pm0_5']:.1f} #/cm³")
            
        except RuntimeError as e:
            print(f"Error: {e}")
            # Get detailed error information
            errors = sensor.error_status_description
            for error_type, description in errors.items():
                print(f"  - {error_type}: {description}")
            
            # Optionally clear errors and try again
            # sensor.clear_device_status()
            
        print("-" * 40)
    time.sleep(1)