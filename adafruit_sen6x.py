# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_sen6x`
================================================================================

CircuitPython driver for the Sensirion SEN6x environmental sensor node


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Link Text <https://www.adafruit.com/product/6331>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import struct
import time

from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    from typing import Any, BinaryIO, Dict, List, Optional, Tuple, Union

    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SEN6x.git"

# I2C addresses for SEN6x series
SEN6X_I2C_ADDRESS = const(0x6B)  # SEN63C, SEN65, SEN66, SEN68
SEN60_I2C_ADDRESS = const(0x6C)  # SEN60 only

# Common commands shared across all SEN6x variants
# Command ID format: 16-bit with built-in 3-bit CRC
_START_MEASUREMENT = const(0x0021)
_STOP_MEASUREMENT = const(0x0104)
_DATA_READY = const(0x0202)
_RESET = const(0xD304)
_SERIAL_NUMBER = const(0xD033)
_VERSION = const(0xD100)
_PRODUCT_NAME = const(0xD014)
_DEVICE_STATUS = const(0xD206)
_CLEAR_DEVICE_STATUS = const(0xD210)
_FAN_CLEANING = const(0x5607)
_VOC_STATE = const(0x6181)
_VOC_TUNING = const(0x60D0)
# CO2 commands (SEN66 specific)
_FORCE_CO2_RECALIBRATION = const(0x6707)
_CO2_AUTO_CALIB = const(0x6711)
_AMBIENT_PRESSURE = const(0x6720)
_SENSOR_ALTITUDE = const(0x6736)
_SHT_HEATER = const(0x6790)

# NOx algorithm commands
_NOX_TUNING = const(0x60E1)

# Temperature configuration commands
_TEMP_OFFSET = const(0x60B2)
_TEMP_ACCELERATION = const(0x6100)

# SEN66-specific commands
_SEN66_READ_MEASUREMENT = const(0x0300)  # Read all measurements
_SEN66_READ_RAW_VALUES = const(0x0405)  # Read raw values
_READ_NUMBER_CONCENTRATION = const(0x0316)

# Command execution times (in seconds)
_TIME_START_MEASUREMENT = const(0.050)  # 50ms
_TIME_STOP_MEASUREMENT = const(1.000)  # 1000ms
_TIME_DATA_READY = const(0.020)  # 20ms
_TIME_READ_MEASUREMENT = const(0.020)  # 20ms
_TIME_STANDARD = const(0.020)  # 20ms for most commands
_TIME_MINIMAL = const(0.001)  # 1ms minimum wait
_TIME_RESET = const(1.200)  # 1200ms for reset
_TIME_SHT_HEATER = const(1.300)  # 1300ms for SHT heater
_TIME_CO2_RECALIBRATION = const(0.500)  # 500ms for CO2 recalibration

# Sensor startup time (maximum)
_SENSOR_STARTUP_TIME = const(1.0)  # 1 second max startup time

# Measurement timing
_FIRST_MEASUREMENT_DELAY = const(1.1)  # 1.1s until first measurement ready
_NOX_STARTUP_TIME = const(11.0)  # 10-11s for NOx sensor initialization
_CO2_STARTUP_TIME = const(6.0)  # 5-6s for CO2 sensor initialization

# Data value indicators
_UNKNOWN_VALUE = const(0xFFFF)  # Unknown value for unsigned 16-bit

# Status register bit positions (for the 32-bit status register)
# Warning bits (upper 16 bits)
_STATUS_SPEED_WARNING = const(21)

# Error bits (lower 16 bits)
_STATUS_CO2_1_ERROR = const(12)
_STATUS_PM_ERROR = const(11)
_STATUS_HCHO_ERROR = const(10)
_STATUS_CO2_2_ERROR = const(9)
_STATUS_GAS_ERROR = const(7)
_STATUS_RHT_ERROR = const(6)
_STATUS_FAN_ERROR = const(4)


class DeviceStatus:
    """Helper class to parse the device status register"""

    def __init__(self, status_data: int) -> None:
        """
        Args:
            status_data: 32-bit status register value
        """
        self._status: int = status_data

    @property
    def speed_warning(self) -> bool:
        """Fan speed out of range warning"""
        return bool(self._status & (1 << _STATUS_SPEED_WARNING))

    @property
    def co2_sensor_1_error(self) -> bool:
        """CO2 sensor 1 error (SEN68 only)

        CO2 values might be unknown or wrong if this flag is set.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_CO2_1_ERROR))

    @property
    def pm_sensor_error(self) -> bool:
        """Particulate matter sensor error (SEN63C, SEN65, SEN66, SEN68)

        PM values might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_PM_ERROR))

    @property
    def hcho_sensor_error(self) -> bool:
        """Formaldehyde sensor error (SEN68 only)

        HCHO values might be unknown or wrong if this flag is set.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_HCHO_ERROR))

    @property
    def co2_sensor_2_error(self) -> bool:
        """CO2 sensor 2 error (SEN66 only)

        CO2 values might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_CO2_2_ERROR))

    @property
    def gas_sensor_error(self) -> bool:
        """VOC/NOx gas sensor error (SEN65, SEN66, SEN68)

        VOC index and NOx index might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_GAS_ERROR))

    @property
    def rht_sensor_error(self) -> bool:
        """Relative humidity and temperature sensor error (SEN63C, SEN65, SEN66, SEN68)

        Temperature and humidity values might be unknown or wrong if this flag is set.
        Other measured values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_RHT_ERROR))

    @property
    def fan_error(self) -> bool:
        """Fan error - fan is mechanically blocked or broken

        Fan is switched on but 0 RPM measured for multiple consecutive intervals.
        All measured values are likely wrong if this error is reported.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_FAN_ERROR))

    @property
    def errors(self) -> bool:
        """Check if any error bits are set"""
        error_mask = (
            (1 << _STATUS_CO2_1_ERROR)
            | (1 << _STATUS_PM_ERROR)
            | (1 << _STATUS_HCHO_ERROR)
            | (1 << _STATUS_CO2_2_ERROR)
            | (1 << _STATUS_GAS_ERROR)
            | (1 << _STATUS_RHT_ERROR)
            | (1 << _STATUS_FAN_ERROR)
        )
        return bool(self._status & error_mask)

    @property
    def warnings(self) -> bool:
        """Check if any warning bits are set"""
        warning_mask = 1 << _STATUS_SPEED_WARNING
        return bool(self._status & warning_mask)

    def __str__(self) -> str:
        """String representation of status"""
        status_items: List[str] = []
        if self.speed_warning:
            status_items.append("Speed Warning")
        if self.co2_sensor_1_error:
            status_items.append("CO2-1 Error")
        if self.pm_sensor_error:
            status_items.append("PM Error")
        if self.hcho_sensor_error:
            status_items.append("HCHO Error")
        if self.co2_sensor_2_error:
            status_items.append("CO2-2 Error")
        if self.gas_sensor_error:
            status_items.append("Gas Error")
        if self.rht_sensor_error:
            status_items.append("RH&T Error")
        if self.fan_error:
            status_items.append("Fan Error")

        if not status_items:
            return "Status: OK"
        return "Status: " + ", ".join(status_items)


class SEN6x:
    """Base class for Sensirion SEN6x environmental sensors"""

    def __init__(self, i2c: I2C, address: int = SEN6X_I2C_ADDRESS) -> None:
        self.i2c_device: I2CDevice = I2CDevice(i2c, address)
        self._serial_number: Optional[str] = None
        self._product_name: Optional[str] = None
        self._measurement_started: bool = False

        # Allow sensor to complete startup
        time.sleep(_SENSOR_STARTUP_TIME)

    def _write_command(
        self, command: int, data: Optional[List[int]] = None, execution_time: float = _TIME_STANDARD
    ) -> None:
        """Write a command to the sensor with optional data

        Args:
            command: 16-bit command ID (already contains 3-bit CRC)
            data: Optional list of 16-bit values to write
            execution_time: Time to wait after command (seconds)
        """

        buffer = struct.pack(">H", command)
        with self.i2c_device as i2c:
            # Write command (MSB first)
            if data is None:
                i2c.write(buffer)
            else:
                for value in data:
                    # Pack 16-bit value
                    value_bytes = struct.pack(">H", value)
                    # Calculate and append CRC
                    crc = self._crc8(value_bytes)
                    buffer += value_bytes + bytes([crc])
                i2c.write(buffer)

        # Wait for command execution
        time.sleep(execution_time)

    def _read_data(self, num_words: int, execution_time: float = _TIME_STANDARD) -> List[int]:
        """Data from sensor after a read command

        Each word is 2 bytes + 1 CRC byte = 3 bytes per word

        Args:
            num_words: Number of 16-bit words to read
            execution_time: Time to wait before reading (seconds)

        Returns:
            List of 16-bit values
        """
        # Wait for command execution before reading
        time.sleep(execution_time)

        buffer = bytearray(num_words * 3)
        with self.i2c_device as i2c:
            i2c.readinto(buffer)

        # Process data and check CRC
        data: List[int] = []
        for i in range(num_words):
            word_start = i * 3
            word_data = buffer[word_start : word_start + 2]
            crc = buffer[word_start + 2]

            # Check CRC
            if self._crc8(word_data) != crc:
                raise RuntimeError(f"CRC check failed for word {i}")

            data.append(struct.unpack(">H", word_data)[0])

        return data

    @staticmethod
    def _crc8(data: bytes) -> int:
        """Calculate CRC8 for Sensirion sensors

        Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
        Initialization: 0xFF
        """
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc

    def reset(self) -> None:
        """Reset the sensor

        After reset, the sensor needs time to start up before accepting commands.
        All configuration parameters are reset to default values.
        """
        self._write_command(_RESET, execution_time=_TIME_RESET)
        self._measurement_started = False
        # Clear cached values
        self._serial_number = None
        self._product_name = None
        # Wait for sensor to restart
        time.sleep(_SENSOR_STARTUP_TIME)

    def start_measurement(self) -> None:
        """Start continuous measurement mode

        Once started, the sensor will continuously update its readings.
        Use data_ready property to check when new data is available.
        """
        if self._measurement_started:
            return
        self._write_command(_START_MEASUREMENT, execution_time=_TIME_START_MEASUREMENT)
        self._measurement_started = True

    def stop_measurement(self) -> None:
        """Stop continuous measurement mode

        Note: This command takes up to 1 second to execute.
        """
        if not self._measurement_started:
            return
        self._write_command(_STOP_MEASUREMENT, execution_time=_TIME_STOP_MEASUREMENT)
        self._measurement_started = False

    @property
    def data_ready(self) -> bool:
        """Check if new measurement data is ready

        Returns:
            bool: True if new data is available
        """
        if not self._measurement_started:
            return False

        self._write_command(_DATA_READY)
        data = self._read_data(1, execution_time=_TIME_DATA_READY)
        # Last bit indicates data ready status
        return bool(data[0] & 0x0001)

    @property
    def serial_number(self) -> str:
        """The sensor serial number as ASCII string (up to 32 characters)"""
        if self._serial_number is None:
            self._write_command(_SERIAL_NUMBER)
            # Serial number is string<32> (16 words max)
            data = self._read_data(16, execution_time=_TIME_STANDARD)
            # Convert to string, removing null termination
            serial_bytes = b""
            for word in data:
                serial_bytes += struct.pack(">H", word)
            self._serial_number = serial_bytes.decode("utf-8").rstrip("\x00")
        return self._serial_number

    @property
    def product_name(self) -> str:
        """The product name (32 bytes)"""
        if self._product_name is None:
            self._write_command(_PRODUCT_NAME)
            # Product name is 32 bytes (16 words)
            data = self._read_data(16, execution_time=_TIME_STANDARD)
            # Convert to string, removing null termination
            name_bytes = b""
            for word in data:
                name_bytes += struct.pack(">H", word)
            self._product_name = name_bytes.decode("utf-8").rstrip("\x00")
        return self._product_name

    @property
    def device_status(self) -> DeviceStatus:
        """The device status register

        Returns:
            DeviceStatus: Object containing parsed status information
        """
        self._write_command(_DEVICE_STATUS)
        # Status register is 32 bits (2 words)
        data = self._read_data(2, execution_time=_TIME_STANDARD)
        # Combine into 32-bit value (MSB first)
        status_value = (data[0] << 16) | data[1]
        return DeviceStatus(status_value)

    def clear_device_status(self) -> None:
        """Clear the device status register

        This clears all error and warning flags. Note that if the error
        condition persists, the flags will be set again. All error flags
        are "sticky" - they remain set even if the error condition goes
        away, until explicitly cleared by this command or a device reset.
        """
        self._write_command(_CLEAR_DEVICE_STATUS, execution_time=_TIME_STANDARD)

    def start_fan_cleaning(self) -> None:
        """Start the fan cleaning procedure

        This accelerates the fan to maximum speed for 10 seconds to blow out
        dust that has accumulated in the fan/sensor housing. This command can
        only be executed when the sensor is in idle mode (not measuring).

        After fan cleaning, wait at least 10 seconds before starting measurement.

        Raises:
            RuntimeError: If sensor is currently measuring
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot start fan cleaning while measuring. Call stop_measurement() first."
            )
        self._write_command(_FAN_CLEANING, execution_time=_TIME_STANDARD)

    @property
    def version(self) -> Tuple[int, int]:
        """Firmware version information

        Returns:
            tuple: (major_version, minor_version)
        """
        self._write_command(_VERSION)  # version command
        data = self._read_data(1, execution_time=_TIME_STANDARD)

        # Version is packed as two bytes in one word
        major = (data[0] >> 8) & 0xFF
        minor = data[0] & 0xFF
        return (major, minor)

    @property
    def sht_heater_measurements(self) -> Dict[str, Optional[float]]:
        """Measurements when SHT heater is active (firmware >= 4.0)

        Returns heating progress measurements. If heating not finished,
        returns None values.

        Returns:
            dict: {'humidity': value or None, 'temperature': value or None}
        """
        self._write_command(_SHT_HEATER)
        data = self._read_data(2, execution_time=_TIME_STANDARD)

        temp_scale = 200.0
        humidity_scale = 100.0

        return {
            "humidity": None if data[0] == _UNKNOWN_VALUE else data[0] / humidity_scale,
            "temperature": None if data[1] == _UNKNOWN_VALUE else data[1] / temp_scale,
        }

    def check_sensor_errors(self) -> None:
        """Check device status and raise exception if critical errors present

        This is a convenience method that checks for sensor errors that would
        make measurements unreliable. It's recommended to call this before
        reading measurements.

        Raises:
            RuntimeError: If critical sensor errors are detected
        """
        status = self.device_status
        if status.fan_error:
            raise RuntimeError("Fan error detected - measurements unreliable")

        errors: List[str] = []
        if status.pm_sensor_error:
            errors.append("PM sensor")
        if status.gas_sensor_error:
            errors.append("Gas sensor")
        if status.rht_sensor_error:
            errors.append("RH&T sensor")

        if errors:
            raise RuntimeError(f"Sensor errors detected: {', '.join(errors)}")

    @property
    def error_status_description(self) -> Dict[str, str]:
        """Human-readable description of current errors and their effects

        Returns:
            dict: Dictionary with error names and their implications
        """
        status = self.device_status
        errors: Dict[str, str] = {}

        if status.fan_error:
            errors["fan"] = "Fan blocked/broken - ALL measurements unreliable"
        if status.pm_sensor_error:
            errors["pm"] = "PM values unreliable, RH&T may be affected"
        if status.gas_sensor_error:
            errors["gas"] = "VOC/NOx indices unreliable, RH&T may be affected"
        if status.rht_sensor_error:
            errors["rht"] = "Temperature/humidity unreliable, other values may be affected"
        if status.co2_sensor_2_error:
            errors["co2"] = "CO2 values unreliable, RH&T may be affected"

        return errors


class SEN66(SEN6x):  # noqa: PLR0904
    """Driver for SEN66 sensor - measures PM, VOC, NOx, CO2, RH, and Temperature"""

    def __init__(self, i2c: I2C, address: int = SEN6X_I2C_ADDRESS) -> None:
        super().__init__(i2c, address)
        self._measurement_data: Optional[Dict[str, Optional[float]]] = None
        self._measurement_time: Optional[float] = None

    def all_measurements(self) -> Dict[str, Optional[float]]:
        """All measurement values from SEN66

        Must be called when sensor is in measurement mode and data is ready.
        Note: CO2 values will be 0xFFFF for first 5-6 seconds after measurement start.
        Note: NOx values will be 0x7FFF for first 10-11 seconds after power-on/reset.

        Returns:
            dict: Dictionary containing:
                - pm1_0: PM1.0 concentration (µg/m³) or None if unknown
                - pm2_5: PM2.5 concentration (µg/m³) or None if unknown
                - pm4_0: PM4.0 concentration (µg/m³) or None if unknown
                - pm10: PM10 concentration (µg/m³) or None if unknown
                - humidity: Relative humidity (%) or None if unknown
                - temperature: Temperature (°C) or None if unknown
                - voc_index: VOC index (0.1-50.0) or None if unknown
                - nox_index: NOx index (0.1-50.0) or None if unknown
                - co2: CO2 concentration (ppm) or None if unknown

        Raises:
            RuntimeError: If sensor is not in measurement mode
        """
        if not self._measurement_started:
            raise RuntimeError(
                "Sensor must be in measurement mode. Call start_measurement() first."
            )

        self._write_command(_SEN66_READ_MEASUREMENT)

        # SEN66 returns 9 values (9 words) - includes CO2
        data = self._read_data(9, execution_time=_TIME_READ_MEASUREMENT)

        # Scale factors from datasheet
        pm_scale = 10.0
        temp_scale = 200.0
        humidity_scale = 100.0
        voc_nox_scale = 10.0

        # Track measurement time for startup detection
        if self._measurement_time is None:
            self._measurement_time = time.monotonic()

        # Process PM values (uint16, 0xFFFF = unknown)
        pm1_0: Optional[float] = None if data[0] == _UNKNOWN_VALUE else data[0] / pm_scale
        pm2_5: Optional[float] = None if data[1] == _UNKNOWN_VALUE else data[1] / pm_scale
        pm4_0: Optional[float] = None if data[2] == _UNKNOWN_VALUE else data[2] / pm_scale
        pm10: Optional[float] = None if data[3] == _UNKNOWN_VALUE else data[3] / pm_scale

        # Process RH&T values (int16, 0x7FFF = unknown)
        humidity: Optional[float] = None if data[4] == _UNKNOWN_VALUE else data[4] / humidity_scale
        temperature: Optional[float] = None if data[5] == _UNKNOWN_VALUE else data[5] / temp_scale

        # Process VOC/NOx indices (int16, 0x7FFF = unknown)
        voc_index: Optional[float] = None if data[6] == _UNKNOWN_VALUE else data[6] / voc_nox_scale
        nox_index: Optional[float] = None if data[7] == _UNKNOWN_VALUE else data[7] / voc_nox_scale

        # Process CO2 (uint16, 0xFFFF = unknown)
        co2: Optional[float] = None if data[8] == _UNKNOWN_VALUE else float(data[8])

        self._measurement_data = {
            "pm1_0": pm1_0,
            "pm2_5": pm2_5,
            "pm4_0": pm4_0,
            "pm10": pm10,
            "humidity": humidity,
            "temperature": temperature,
            "voc_index": voc_index,
            "nox_index": nox_index,
            "co2": co2,
        }

        return self._measurement_data

    def _check_measurements(self) -> None:
        """Ensure measurements have been read"""
        if self._measurement_data is None:
            self._measurement_data = self.all_measurements()

    def raw_values(self) -> Dict[str, Optional[float]]:
        """Raw sensor values from SEN66

        Returns raw sensor readings without index calculations.

        Returns:
            dict: Dictionary containing:
                - raw_humidity: Raw humidity (%)
                - raw_temperature: Raw temperature (°C)
                - raw_voc: Raw VOC ticks (no scale)
                - raw_nox: Raw NOx ticks (no scale)
                - raw_co2: Raw CO2 concentration (ppm, updated every 5s)
        """
        if not self._measurement_started:
            raise RuntimeError(
                "Sensor must be in measurement mode. Call start_measurement() first."
            )

        self._write_command(_SEN66_READ_RAW_VALUES)
        data = self._read_data(5, execution_time=_TIME_READ_MEASUREMENT)

        temp_scale = 200.0
        humidity_scale = 100.0

        return {
            "raw_humidity": None if data[0] == _UNKNOWN_VALUE else data[0] / humidity_scale,
            "raw_temperature": None if data[1] == _UNKNOWN_VALUE else data[1] / temp_scale,
            "raw_voc": None if data[2] == _UNKNOWN_VALUE else float(data[2]),
            "raw_nox": None if data[3] == _UNKNOWN_VALUE else float(data[3]),
            "raw_co2": None if data[4] == _UNKNOWN_VALUE else float(data[4]),
        }

    def number_concentration(self) -> Dict[str, Optional[float]]:
        """Particle number concentration values

        Returns:
            dict: Dictionary containing number concentrations (particles/cm³):
                - nc_pm0_5: PM0.5 number concentration
                - nc_pm1_0: PM1.0 number concentration
                - nc_pm2_5: PM2.5 number concentration
                - nc_pm4_0: PM4.0 number concentration
                - nc_pm10: PM10 number concentration
        """
        if not self._measurement_started:
            raise RuntimeError(
                "Sensor must be in measurement mode. Call start_measurement() first."
            )

        self._write_command(_READ_NUMBER_CONCENTRATION)
        data = self._read_data(5, execution_time=_TIME_READ_MEASUREMENT)

        nc_scale = 10.0

        return {
            "nc_pm0_5": None if data[0] == _UNKNOWN_VALUE else data[0] / nc_scale,
            "nc_pm1_0": None if data[1] == _UNKNOWN_VALUE else data[1] / nc_scale,
            "nc_pm2_5": None if data[2] == _UNKNOWN_VALUE else data[2] / nc_scale,
            "nc_pm4_0": None if data[3] == _UNKNOWN_VALUE else data[3] / nc_scale,
            "nc_pm10": None if data[4] == _UNKNOWN_VALUE else data[4] / nc_scale,
        }

    def temperature_offset(
        self, offset: float = 0.0, slope: float = 0.0, time_constant: int = 0, slot: int = 0
    ) -> None:
        """Temperature offset parameters for design-in compensation

        Compensated temperature = ambient_temp + (slope * ambient_temp) + offset

        Args:
            offset: Constant temperature offset in °C (default: 0.0)
            slope: Temperature dependent offset factor (default: 0.0)
            time_constant: Time constant in seconds for applying changes (default: 0 = immediate)
            slot: Offset slot to modify (0-4, default: 0)

        Note: Configuration is volatile and reset to defaults after power cycle
        """
        if not 0 <= slot <= 4:
            raise ValueError("Slot must be 0-4")

        # Scale factors - these are signed values
        offset_scaled = int(offset * 200)
        slope_scaled = int(slope * 10000)

        # Convert signed to unsigned for I2C transmission
        offset_scaled = offset_scaled & 0xFFFF
        slope_scaled = slope_scaled & 0xFFFF

        data = [offset_scaled, slope_scaled, time_constant, slot]
        self._write_command(_TEMP_OFFSET, data=data, execution_time=_TIME_STANDARD)

    def temperature_acceleration(
        self, k: float = 10.0, p: float = 10.0, t1: float = 10.0, t2: float = 10.0
    ) -> None:
        """Temperature acceleration parameters for RH/T engine

        Overwrites default temperature acceleration parameters.

        Args:
            k: Filter constant K (default: 10.0, actual = value/10)
            p: Filter constant P (default: 10.0, actual = value/10)
            t1: Time constant T1 in seconds (default: 10.0, actual = value/10)
            t2: Time constant T2 in seconds (default: 10.0, actual = value/10)

        Note: Configuration is volatile and reset to defaults after power cycle.
        Must be called in idle mode.
        """
        if self._measurement_started:
            raise RuntimeError("Cannot set temperature acceleration while measuring.")

        # Scale factors (multiply by 10 for protocol)
        k_scaled = int(k * 10)
        p_scaled = int(p * 10)
        t1_scaled = int(t1 * 10)
        t2_scaled = int(t2 * 10)

        data = [k_scaled, p_scaled, t1_scaled, t2_scaled]
        self._write_command(_TEMP_ACCELERATION, data=data, execution_time=_TIME_STANDARD)

    def force_co2_recalibration(self, target_co2_ppm: int) -> Optional[int]:
        """Perform forced CO2 recalibration (FRC)

        Forces the CO2 sensor to recalibrate to a known reference concentration.
        This should be done when the sensor is in a controlled environment with
        a known CO2 concentration (e.g., fresh outdoor air at ~420 ppm).

        Args:
            target_co2_ppm: Known CO2 concentration in ppm at current location

        Returns:
            int: CO2 correction applied in ppm, or None if recalibration failed

        Raises:
            RuntimeError: If sensor is currently measuring

        Note: Sensor must be in idle mode. Wait at least 1000ms after power-on
        or 600ms after stop_measurement() before calling this.
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot recalibrate CO2 while measuring. Call stop_measurement() first."
            )

        # Send target CO2 concentration
        self._write_command(
            _FORCE_CO2_RECALIBRATION, data=[target_co2_ppm], execution_time=_TIME_CO2_RECALIBRATION
        )

        # Read correction value
        data = self._read_data(1, execution_time=0)  # No additional wait, already waited 500ms

        # Check if recalibration failed
        if data[0] == 0xFFFF:
            return None

        # Calculate actual correction: correction = return_value - 0x8000
        correction = data[0] - 0x8000
        return correction

    @property
    def co2_automatic_self_calibration(self) -> bool:
        """CO2 sensor automatic self-calibration (ASC) status

        Returns:
            bool: True if ASC is enabled, False if disabled
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot read CO2 ASC while measuring. Call stop_measurement() first."
            )

        self._write_command(_CO2_AUTO_CALIB)
        data = self._read_data(1, execution_time=_TIME_STANDARD)

        # Data format: [padding_byte, status_byte] packed in one word
        # Extract status byte (LSB)
        return bool(data[0] & 0xFF)

    @co2_automatic_self_calibration.setter
    def co2_automatic_self_calibration(self, enabled: bool) -> None:
        """CO2 sensor automatic self-calibration (ASC) status

        ASC assumes the sensor is exposed to fresh air (~400 ppm) at least
        once every few days. Enable for office/home use, disable for
        greenhouses or continuously occupied spaces.

        Args:
            enabled: True to enable ASC, False to disable

        Note: Default is enabled. Setting is volatile (reset on power cycle).
        """
        if self._measurement_started:
            raise RuntimeError("Cannot set CO2 ASC while measuring. Call stop_measurement() first.")

        # Pack padding byte (0x00) and status byte into one word
        status_word = 0x0001 if enabled else 0x0000
        self._write_command(_CO2_AUTO_CALIB, data=[status_word], execution_time=_TIME_STANDARD)

    @property
    def ambient_pressure(self) -> int:
        """Ambient pressure used for CO2 compensation

        Returns:
            int: Current ambient pressure in hPa (hectopascals)
        """
        self._write_command(_AMBIENT_PRESSURE)
        data = self._read_data(1, execution_time=_TIME_STANDARD)
        return data[0]

    @ambient_pressure.setter
    def ambient_pressure(self, pressure_hpa: int) -> None:
        """Ambient pressure for CO2 compensation

        Use this for applications with significant pressure changes.
        Setting pressure overrides any altitude-based compensation.

        Args:
            pressure_hpa: Ambient pressure in hPa (700-1200, default: 1013)

        Raises:
            ValueError: If pressure is outside valid range

        Note: Setting is volatile (reset to 1013 hPa on power cycle).
        """
        if not 700 <= pressure_hpa <= 1200:
            raise ValueError("Ambient pressure must be 700-1200 hPa")

        self._write_command(_AMBIENT_PRESSURE, data=[pressure_hpa], execution_time=_TIME_STANDARD)

    @property
    def sensor_altitude(self) -> int:
        """Sensor altitude used for CO2 compensation

        Returns:
            int: Current sensor altitude in meters above sea level
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot read altitude while measuring. Call stop_measurement() first."
            )

        self._write_command(_SENSOR_ALTITUDE)
        data = self._read_data(1, execution_time=_TIME_STANDARD)
        return data[0]

    @sensor_altitude.setter
    def sensor_altitude(self, altitude_m: int) -> None:
        """Sensor altitude for CO2 compensation

        Alternative to setting ambient pressure directly.
        The sensor will calculate pressure based on altitude.

        Args:
            altitude_m: Altitude in meters (0-3000, default: 0)

        Raises:
            ValueError: If altitude is outside valid range
            RuntimeError: If sensor is currently measuring

        Note: Setting is volatile (reset to 0m on power cycle).
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot set altitude while measuring. Call stop_measurement() first."
            )

        if not 0 <= altitude_m <= 3000:
            raise ValueError("Altitude must be 0-3000 meters")

        self._write_command(_SENSOR_ALTITUDE, data=[altitude_m], execution_time=_TIME_STANDARD)

    @property
    def voc_algorithm_state(self) -> bytes:
        """VOC algorithm state for backup/restore

        Can be called in either idle or measurement mode. In measurement mode,
        returns the current state. In idle mode, returns the state from when
        measurement was stopped.

        Returns:
            bytes: 8-byte algorithm state that can be restored later
        """
        self._write_command(_VOC_STATE)
        data = self._read_data(4, execution_time=_TIME_STANDARD)  # 4 words = 8 bytes

        # Convert words to bytes
        state = b""
        for word in data:
            state += struct.pack(">H", word)
        return state

    @voc_algorithm_state.setter
    def voc_algorithm_state(self, state: bytes) -> None:
        """Restore VOC algorithm state from backup

        Allows skipping the initial VOC learning phase after power cycle.
        Must be called in idle mode before starting measurement.

        Args:
            state: 8-byte algorithm state from get_voc_algorithm_state()

        Note: Only works in idle mode, applied when measurement starts
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot set VOC state while measuring. Call stop_measurement() first."
            )

        if len(state) != 8:
            raise ValueError("State must be exactly 8 bytes")

        # Convert bytes to words
        data: List[int] = []
        for i in range(0, 8, 2):
            data.append(struct.unpack(">H", state[i : i + 2])[0])

        self._write_command(_VOC_STATE, data=data, execution_time=_TIME_STANDARD)

    @property
    def voc_algorithm(self) -> Dict[str, int]:
        """VOC algorithm tuning parameters

        Returns:
            dict: Current VOC algorithm parameters
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot read VOC tuning while measuring. Call stop_measurement() first."
            )

        self._write_command(_VOC_TUNING)
        data = self._read_data(6, execution_time=_TIME_STANDARD)

        return {
            "index_offset": data[0],
            "learning_time_offset_hours": data[1],
            "learning_time_gain_hours": data[2],
            "gating_max_duration_minutes": data[3],
            "std_initial": data[4],
            "gain_factor": data[5],
        }

    def voc_algorithm_tuning(  # noqa: PLR0913 PLR0917
        self,
        index_offset: int = 100,
        learning_time_offset_hours: int = 12,
        learning_time_gain_hours: int = 12,
        gating_max_duration_minutes: int = 180,
        std_initial: int = 50,
        gain_factor: int = 230,
    ) -> None:
        """VOC algorithm tuning parameters

        Args:
            index_offset: VOC index for average conditions (1-250, default: 100)
            learning_time_offset_hours: Time constant for offset learning (1-1000, default: 12)
            learning_time_gain_hours: Time constant for gain learning (1-1000, default: 12)
            gating_max_duration_minutes: Max gating duration (0-3000, default: 180, 0=disabled)
            std_initial: Initial standard deviation (10-5000, default: 50)
            gain_factor: Output gain factor (1-1000, default: 230)

        Note: Configuration is volatile and reset to defaults after power cycle
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot set VOC tuning while measuring. Call stop_measurement() first."
            )

        # Validate ranges
        if not 1 <= index_offset <= 250:
            raise ValueError("index_offset must be 1-250")
        if not 1 <= learning_time_offset_hours <= 1000:
            raise ValueError("learning_time_offset_hours must be 1-1000")
        if not 1 <= learning_time_gain_hours <= 1000:
            raise ValueError("learning_time_gain_hours must be 1-1000")
        if not 0 <= gating_max_duration_minutes <= 3000:
            raise ValueError("gating_max_duration_minutes must be 0-3000")
        if not 10 <= std_initial <= 5000:
            raise ValueError("std_initial must be 10-5000")
        if not 1 <= gain_factor <= 1000:
            raise ValueError("gain_factor must be 1-1000")

        data = [
            index_offset,
            learning_time_offset_hours,
            learning_time_gain_hours,
            gating_max_duration_minutes,
            std_initial,
            gain_factor,
        ]
        self._write_command(_VOC_TUNING, data=data, execution_time=_TIME_STANDARD)

    @property
    def nox_algorithm(self) -> Dict[str, int]:
        """NOx algorithm tuning parameters

        Returns:
            dict: Current NOx algorithm parameters
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot read NOx tuning while measuring. Call stop_measurement() first."
            )

        self._write_command(_NOX_TUNING)
        data = self._read_data(6, execution_time=_TIME_STANDARD)

        return {
            "index_offset": data[0],
            "learning_time_offset_hours": data[1],
            "learning_time_gain_hours": data[2],  # No effect for NOx
            "gating_max_duration_minutes": data[3],
            "std_initial": data[4],  # No effect for NOx
            "gain_factor": data[5],
        }

    def nox_algorithm_tuning(
        self,
        index_offset: int = 1,
        learning_time_offset_hours: int = 12,
        gating_max_duration_minutes: int = 720,
        gain_factor: int = 230,
    ) -> None:
        """NOx algorithm tuning parameters

        Args:
            index_offset: NOx index for average conditions (1-250, default: 1)
            learning_time_offset_hours: Time constant for offset learning (1-1000, default: 12)
            gating_max_duration_minutes: Max gating duration (0-3000, default: 720, 0=disabled)
            gain_factor: Output gain factor (1-1000, default: 230)

        Note: learning_time_gain_hours is fixed at 12, std_initial is fixed at 50 for NOx.
        Configuration is volatile and reset to defaults after power cycle.
        """
        if self._measurement_started:
            raise RuntimeError(
                "Cannot set NOx tuning while measuring. Call stop_measurement() first."
            )

        # Validate ranges
        if not 1 <= index_offset <= 250:
            raise ValueError("index_offset must be 1-250")
        if not 1 <= learning_time_offset_hours <= 1000:
            raise ValueError("learning_time_offset_hours must be 1-1000")
        if not 0 <= gating_max_duration_minutes <= 3000:
            raise ValueError("gating_max_duration_minutes must be 0-3000")
        if not 1 <= gain_factor <= 1000:
            raise ValueError("gain_factor must be 1-1000")

        # Fixed parameters for NOx
        learning_time_gain_hours = 12  # Must be 12 for NOx
        std_initial = 50  # Must be 50 for NOx

        data = [
            index_offset,
            learning_time_offset_hours,
            learning_time_gain_hours,
            gating_max_duration_minutes,
            std_initial,
            gain_factor,
        ]
        self._write_command(_NOX_TUNING, data=data, execution_time=_TIME_STANDARD)

    @property
    def temperature(self) -> Optional[float]:
        """Temperature in Celsius"""
        self.all_measurements()
        return self._measurement_data["temperature"] if self._measurement_data else None

    @property
    def humidity(self) -> Optional[float]:
        """Relative humidity in percent"""
        self.all_measurements()
        return self._measurement_data["humidity"] if self._measurement_data else None

    @property
    def pm2_5(self) -> Optional[float]:
        """PM2.5 concentration in µg/m³"""
        self.all_measurements()
        return self._measurement_data["pm2_5"] if self._measurement_data else None

    @property
    def voc_index(self) -> Optional[float]:
        """VOC index (0.1-50.0)"""
        self.all_measurements()
        return self._measurement_data["voc_index"] if self._measurement_data else None

    @property
    def nox_index(self) -> Optional[float]:
        """NOx index (0.1-50.0)"""
        self.all_measurements()
        return self._measurement_data["nox_index"] if self._measurement_data else None

    @property
    def co2(self) -> Optional[float]:
        """CO2 concentration in ppm"""
        self.all_measurements()
        return self._measurement_data["co2"] if self._measurement_data else None
