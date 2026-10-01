# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_sen6x`
================================================================================

CircuitPython driver for the Sensirion SEN6x environmental sensor node
(SEN60, SEN62, SEN63C, SEN65, SEN66, SEN68 and SEN69C)


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
SEN6X_I2C_ADDRESS = const(0x6B)  # SEN62, SEN63C, SEN65, SEN66, SEN68, SEN69C
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
_READ_NUMBER_CONCENTRATION = const(0x0316)

# Temperature configuration commands
_TEMP_OFFSET = const(0x60B2)
_TEMP_ACCELERATION = const(0x6100)

# SHT heater commands
_ACTIVATE_SHT_HEATER = const(0x6765)
_SHT_HEATER = const(0x6790)

# VOC/NOx algorithm commands (SEN65, SEN66, SEN68, SEN69C)
_VOC_STATE = const(0x6181)
_VOC_TUNING = const(0x60D0)
_NOX_TUNING = const(0x60E1)

# CO2 commands (SEN63C, SEN66, SEN69C)
_FORCE_CO2_RECALIBRATION = const(0x6707)
_CO2_FACTORY_RESET = const(0x6754)
_CO2_AUTO_CALIB = const(0x6711)
_AMBIENT_PRESSURE = const(0x6720)
_SENSOR_ALTITUDE = const(0x6736)

# Model-specific read measured values commands
_SEN62_READ_MEASUREMENT = const(0x04A3)
_SEN63C_READ_MEASUREMENT = const(0x0471)
_SEN65_READ_MEASUREMENT = const(0x0446)
_SEN66_READ_MEASUREMENT = const(0x0300)
_SEN68_READ_MEASUREMENT = const(0x0467)
_SEN69C_READ_MEASUREMENT = const(0x04B5)

# Model-specific read measured raw values commands
_SEN62_SEN63C_READ_RAW_VALUES = const(0x0492)
_SEN65_SEN68_SEN69C_READ_RAW_VALUES = const(0x0455)
_SEN66_READ_RAW_VALUES = const(0x0405)

# SEN60 commands (separate command set, I2C address 0x6C)
_SEN60_START_MEASUREMENT = const(0x2152)
_SEN60_STOP_MEASUREMENT = const(0x3F86)
_SEN60_DATA_READY = const(0xE4B8)
_SEN60_READ_MEASUREMENT = const(0xEC05)
_SEN60_SERIAL_NUMBER = const(0x3682)
_SEN60_DEVICE_STATUS = const(0xE00B)
_SEN60_RESET = const(0x3F8D)
_SEN60_FAN_CLEANING = const(0x3730)

# Command execution times (in seconds)
_TIME_START_MEASUREMENT = const(0.050)  # 50ms
_TIME_STOP_MEASUREMENT = const(1.400)  # 1400ms
_TIME_DATA_READY = const(0.020)  # 20ms
_TIME_READ_MEASUREMENT = const(0.020)  # 20ms
_TIME_STANDARD = const(0.020)  # 20ms for most commands
_TIME_MINIMAL = const(0.001)  # 1ms minimum wait
_TIME_RESET = const(1.200)  # 1200ms for reset
_TIME_SHT_HEATER = const(1.300)  # 1300ms for SHT heater on older firmware
_TIME_CO2_RECALIBRATION = const(0.500)  # 500ms for CO2 recalibration
_TIME_CO2_FACTORY_RESET = const(1.400)  # 1400ms for CO2 sensor factory reset
_TIME_SEN60_COMMAND = const(0.001)  # 1ms for SEN60 commands
_TIME_SEN60_STOP_MEASUREMENT = const(1.000)  # 1000ms for SEN60 stop measurement

# Sensor startup time (maximum)
_SENSOR_STARTUP_TIME = const(1.0)  # 1 second max startup time

# Measurement timing
_FIRST_MEASUREMENT_DELAY = const(1.1)  # 1.1s until first measurement ready
_NOX_STARTUP_TIME = const(11.0)  # 10-11s for NOx sensor initialization
_CO2_STARTUP_TIME = const(6.0)  # 5-6s for CO2 sensor initialization

# Data value indicators
_UNKNOWN_UINT16 = const(0xFFFF)  # Unknown value for unsigned 16-bit
_UNKNOWN_INT16 = const(0x7FFF)  # Unknown value for signed 16-bit

# Word layouts returned by the read commands: (key, is_int16, scale factor).
# int16 words report unknown as 0x7FFF, uint16 words report unknown as 0xFFFF.
_PM_RHT_FIELDS = (
    ("pm1_0", False, 10),
    ("pm2_5", False, 10),
    ("pm4_0", False, 10),
    ("pm10", False, 10),
    ("humidity", True, 100),
    ("temperature", True, 200),
)
_VOC_NOX_FIELDS = (("voc_index", True, 10), ("nox_index", True, 10))
_HCHO_FIELDS = (("hcho", False, 10),)
_CO2_INT16_FIELDS = (("co2", True, 1),)  # SEN63C, SEN69C
_CO2_UINT16_FIELDS = (("co2", False, 1),)  # SEN66
_RAW_RHT_FIELDS = (("raw_humidity", True, 100), ("raw_temperature", True, 200))
_RAW_VOC_NOX_FIELDS = (("raw_voc", False, 1), ("raw_nox", False, 1))
_RAW_CO2_FIELDS = (("raw_co2", False, 1),)
_SHT_HEATER_FIELDS = (("humidity", True, 100), ("temperature", True, 200))
_NUMBER_CONCENTRATION_FIELDS = (
    ("nc_pm0_5", False, 10),
    ("nc_pm1_0", False, 10),
    ("nc_pm2_5", False, 10),
    ("nc_pm4_0", False, 10),
    ("nc_pm10", False, 10),
)

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

# SEN60 16-bit status register (fan error is also bit 4)
_SEN60_STATUS_SPEED_WARNING = const(1)


def _convert_word(word: int, is_int16: bool, scale: int) -> Optional[float]:
    """Convert a raw 16-bit word to a scaled value, or None if unknown"""
    if is_int16:
        if word == _UNKNOWN_INT16:
            return None
        if word & 0x8000:
            word -= 0x10000
    elif word == _UNKNOWN_UINT16:
        return None
    return word / scale


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
        """CO2 sensor 1 error (SEN63C, SEN69C)

        CO2 values might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_CO2_1_ERROR))

    @property
    def pm_sensor_error(self) -> bool:
        """Particulate matter sensor error (all SEN6x)

        PM values might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_PM_ERROR))

    @property
    def hcho_sensor_error(self) -> bool:
        """Formaldehyde sensor error (SEN68, SEN69C)

        HCHO values might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
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
        """VOC/NOx gas sensor error (SEN65, SEN66, SEN68, SEN69C)

        VOC index and NOx index might be unknown or wrong if this flag is set.
        RH and temperature values might be out of spec due to compensation algorithms.
        This is a sticky error that persists until cleared.
        """
        return bool(self._status & (1 << _STATUS_GAS_ERROR))

    @property
    def rht_sensor_error(self) -> bool:
        """Relative humidity and temperature sensor error (all SEN6x)

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


class _SEN6xBase:
    """Shared I2C protocol and measurement handling for the SEN6x family

    Not intended to be used directly, use :class:`SEN6x` subclasses or :class:`SEN60`.
    """

    # Overridden by subclasses: the SEN60 uses its own command set and timings
    _START_COMMAND: int = _START_MEASUREMENT
    _STOP_COMMAND: int = _STOP_MEASUREMENT
    _DATA_READY_COMMAND: int = _DATA_READY
    _DATA_READY_MASK: int = 0x0001
    _FAN_CLEANING_COMMAND: int = _FAN_CLEANING
    _TIME_COMMAND: float = _TIME_STANDARD
    _TIME_START: float = _TIME_START_MEASUREMENT
    _TIME_STOP: float = _TIME_STOP_MEASUREMENT
    _MEASUREMENT_COMMAND: Optional[int] = None
    _MEASUREMENT_FIELDS: Tuple = ()
    # CO2 conditioning period after measurement start (SEN63C, SEN69C)
    _CO2_CONDITIONING_TIME: float = 0

    def __init__(self, i2c: I2C, address: int, startup_delay: float) -> None:
        self.i2c_device: I2CDevice = I2CDevice(i2c, address)
        self._serial_number: Optional[str] = None
        self._measurement_started: bool = False
        self._measurement_data: Optional[Dict[str, Optional[float]]] = None
        self._measurement_start_time: Optional[float] = None

        # Allow sensor to complete startup
        time.sleep(startup_delay)

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

    def _read_values(self, command: int, fields: Tuple) -> Dict[str, Optional[float]]:
        """Send a read command and decode the returned words using a field layout"""
        self._write_command(command, execution_time=0)
        data = self._read_data(len(fields), execution_time=self._TIME_COMMAND)
        return {
            name: _convert_word(word, is_int16, scale)
            for (name, is_int16, scale), word in zip(fields, data)
        }

    def _require_measuring(self) -> None:
        """Raise if the sensor is not in measurement mode"""
        if not self._measurement_started:
            raise RuntimeError(
                "Sensor must be in measurement mode. Call start_measurement() first."
            )

    def _require_idle(self, action: str) -> None:
        """Raise if the sensor is in measurement mode"""
        if self._measurement_started:
            raise RuntimeError(f"Cannot {action} while measuring. Call stop_measurement() first.")

    def _check_co2_conditioning(self, action: str) -> None:
        """Raise if still inside the SEN63C/SEN69C CO2 conditioning period

        Interrupting the conditioning that runs during the first 24 seconds of a
        measurement causes a CO2-1 sensor error.
        """
        if not self._CO2_CONDITIONING_TIME or self._measurement_start_time is None:
            return
        remaining = self._measurement_start_time + self._CO2_CONDITIONING_TIME - time.monotonic()
        if remaining > 0:
            # Single f-string: CircuitPython can't join adjacent f-strings
            raise RuntimeError(
                f"Cannot {action} during CO2 sensor conditioning, wait {remaining:.1f}s more"
            )

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
                    crc <<= 1
                crc &= 0xFF
        return crc

    def start_measurement(self) -> None:
        """Start continuous measurement mode

        Once started, the sensor will continuously update its readings.
        Use data_ready property to check when new data is available.

        Note: SEN63C and SEN69C condition their CO2 sensor for 24 seconds after
        starting a measurement. A measurement may be stopped during that time, but
        must not be started again until the 24 seconds have passed.

        Raises:
            RuntimeError: If restarted during the SEN63C/SEN69C CO2 conditioning period
        """
        if self._measurement_started:
            return
        self._check_co2_conditioning("restart measurement")
        self._write_command(self._START_COMMAND, execution_time=self._TIME_START)
        self._measurement_started = True
        self._measurement_start_time = time.monotonic()

    def stop_measurement(self) -> None:
        """Stop continuous measurement mode

        Note: This command takes up to 1.4 seconds (1 second on SEN60) to execute.
        """
        if not self._measurement_started:
            return
        self._write_command(self._STOP_COMMAND, execution_time=self._TIME_STOP)
        self._measurement_started = False

    @property
    def data_ready(self) -> bool:
        """Check if new measurement data is ready

        Returns:
            bool: True if new data is available
        """
        if not self._measurement_started:
            return False

        self._write_command(self._DATA_READY_COMMAND, execution_time=0)
        data = self._read_data(1, execution_time=self._TIME_COMMAND)
        return bool(data[0] & self._DATA_READY_MASK)

    def start_fan_cleaning(self) -> None:
        """Start the fan cleaning procedure

        This accelerates the fan to maximum speed for 10 seconds to blow out
        dust that has accumulated in the fan/sensor housing. This command can
        only be executed when the sensor is in idle mode (not measuring).

        After fan cleaning, wait at least 10 seconds before starting measurement.

        Raises:
            RuntimeError: If sensor is currently measuring
        """
        self._require_idle("start fan cleaning")
        self._write_command(self._FAN_CLEANING_COMMAND, execution_time=self._TIME_COMMAND)

    @property
    def device_status(self) -> DeviceStatus:
        """The device status register"""
        raise NotImplementedError

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
        if status.co2_sensor_1_error or status.co2_sensor_2_error:
            errors.append("CO2 sensor")
        if status.hcho_sensor_error:
            errors.append("HCHO sensor")

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
        if status.co2_sensor_1_error or status.co2_sensor_2_error:
            errors["co2"] = "CO2 values unreliable, RH&T may be affected"
        if status.hcho_sensor_error:
            errors["hcho"] = "Formaldehyde values unreliable, RH&T may be affected"

        return errors

    def all_measurements(self) -> Dict[str, Optional[float]]:
        """All measurement values for this sensor model

        Must be called when sensor is in measurement mode. Use :attr:`data_ready`
        to check if new data is available, otherwise the previous values are returned.
        Unknown values (e.g. while a sensor is still starting up) are None.

        Keys present depend on the model:

        - pm1_0, pm2_5, pm4_0, pm10: Mass concentration (µg/m³), all models
        - humidity: Relative humidity (%), all models except SEN60
        - temperature: Temperature (°C), all models except SEN60
        - voc_index, nox_index: VOC/NOx index (1.0-500.0), SEN65, SEN66, SEN68, SEN69C
        - hcho: Formaldehyde concentration (ppb), SEN68, SEN69C
        - co2: CO2 concentration (ppm), SEN63C, SEN66, SEN69C
        - nc_pm0_5, nc_pm1_0, nc_pm2_5, nc_pm4_0, nc_pm10: Number concentration
          (particles/cm³), SEN60 only (use :meth:`number_concentration` on other models)

        Returns:
            dict: Measurement name to value or None if unknown

        Raises:
            RuntimeError: If sensor is not in measurement mode
        """
        if self._MEASUREMENT_COMMAND is None:
            raise NotImplementedError("Use the model-specific class, e.g. SEN66")
        self._require_measuring()
        self._measurement_data = self._read_values(
            self._MEASUREMENT_COMMAND, self._MEASUREMENT_FIELDS
        )
        return self._measurement_data

    @property
    def pm1_0(self) -> Optional[float]:
        """PM1.0 concentration in µg/m³"""
        self.all_measurements()
        return self._measurement_data["pm1_0"] if self._measurement_data else None

    @property
    def pm2_5(self) -> Optional[float]:
        """PM2.5 concentration in µg/m³"""
        self.all_measurements()
        return self._measurement_data["pm2_5"] if self._measurement_data else None

    @property
    def pm4_0(self) -> Optional[float]:
        """PM4.0 concentration in µg/m³"""
        self.all_measurements()
        return self._measurement_data["pm4_0"] if self._measurement_data else None

    @property
    def pm10(self) -> Optional[float]:
        """PM10 concentration in µg/m³"""
        self.all_measurements()
        return self._measurement_data["pm10"] if self._measurement_data else None


class SEN6x(_SEN6xBase):  # noqa: PLR0904
    """Base class for Sensirion SEN6x environmental sensors

    Each model uses different commands and data layouts for reading measurements.
    Using ``SEN6x`` directly detects the model from its product name, so
    :meth:`all_measurements`, :meth:`raw_values` and the common properties work
    for any model. For model-specific features (VOC/NOx, CO2 and formaldehyde
    commands and properties) use :func:`create_sensor`, or the subclass matching
    your sensor: :class:`SEN62`, :class:`SEN63C`, :class:`SEN65`, :class:`SEN66`,
    :class:`SEN68` or :class:`SEN69C`.

    Args:
        i2c: The I2C bus the sensor is connected to
        address: The I2C address of the sensor (default: 0x6B)
        startup_delay: Seconds to wait for the sensor to start up (default: 1.0).
            Can be set to 0 if the sensor has already been powered for a while.
    """

    _RAW_VALUES_COMMAND: Optional[int] = None
    _RAW_VALUES_FIELDS: Tuple = ()
    # First firmware version supporting Get SHT Heater Measurements
    _SHT_HEATER_POLL_FIRMWARE: Tuple[int, int] = (0, 0)

    def __init__(
        self,
        i2c: I2C,
        address: int = SEN6X_I2C_ADDRESS,
        startup_delay: float = _SENSOR_STARTUP_TIME,
    ) -> None:
        super().__init__(i2c, address, startup_delay)
        self._product_name: Optional[str] = None
        self._firmware_version: Optional[Tuple[int, int]] = None
        self._model_class: Optional[type] = None
        if type(self) is SEN6x:
            # Generic instance: use the data layouts of the detected model
            model = self.model_class
            self._MEASUREMENT_COMMAND = model._MEASUREMENT_COMMAND
            self._MEASUREMENT_FIELDS = model._MEASUREMENT_FIELDS
            self._RAW_VALUES_COMMAND = model._RAW_VALUES_COMMAND
            self._RAW_VALUES_FIELDS = model._RAW_VALUES_FIELDS
            self._SHT_HEATER_POLL_FIRMWARE = model._SHT_HEATER_POLL_FIRMWARE
            self._CO2_CONDITIONING_TIME = model._CO2_CONDITIONING_TIME

    def _read_string(self, command: int) -> str:
        """Read a null-terminated string<32> (16 words)"""
        self._write_command(command)
        data = self._read_data(16, execution_time=_TIME_STANDARD)
        raw = b"".join(struct.pack(">H", word) for word in data)
        return raw.split(b"\x00")[0].decode("utf-8")

    def reset(self) -> None:
        """Reset the sensor

        Has the same effect as a power cycle. Measurement is stopped first if running.
        After reset, the sensor needs time to start up before accepting commands.
        All volatile configuration parameters are reset to default values.
        """
        self.stop_measurement()
        self._write_command(_RESET, execution_time=_TIME_RESET)
        # Clear cached values
        self._serial_number = None
        self._product_name = None
        self._measurement_data = None
        # Wait for sensor to restart
        time.sleep(_SENSOR_STARTUP_TIME)

    @property
    def serial_number(self) -> str:
        """The sensor serial number as ASCII string (up to 32 characters)"""
        if self._serial_number is None:
            self._serial_number = self._read_string(_SERIAL_NUMBER)
        return self._serial_number

    @property
    def product_name(self) -> str:
        """The product name as ASCII string (up to 32 characters)"""
        if self._product_name is None:
            self._product_name = self._read_string(_PRODUCT_NAME)
        return self._product_name

    @property
    def model_class(self) -> type:
        """The driver class matching this sensor, e.g. :class:`SEN66`

        Detected from the product name. Some sensors report an empty product name,
        in which case each model's Read Measured Values command is tried and the
        one the sensor answers with valid data is used.

        Raises:
            RuntimeError: If the sensor model cannot be identified
        """
        if self._model_class is None:
            name = self.product_name.strip().upper()
            for model, cls in _SEN6X_MODELS:
                if name.startswith(model):
                    self._model_class = cls
                    break
            else:
                for _, cls in _SEN6X_MODELS:
                    if self._command_supported(
                        cls._MEASUREMENT_COMMAND, len(cls._MEASUREMENT_FIELDS)
                    ):
                        self._model_class = cls
                        break
                else:
                    raise RuntimeError(f"Unrecognised SEN6x product name: {name!r}")
        return self._model_class

    def _command_supported(self, command: int, num_words: int) -> bool:
        """True if the sensor answers a read command with CRC-valid data

        Unsupported commands are NACKed, which reads back as 0xFF bytes on some
        hosts (failing the CRC check) and raises OSError on others.
        """
        try:
            self._write_command(command, execution_time=0)
            self._read_data(num_words, execution_time=self._TIME_COMMAND)
        except (OSError, RuntimeError):
            return False
        return True

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

    def clear_device_status(self) -> DeviceStatus:
        """Read and clear the device status register

        This clears all error and warning flags. Note that if the error
        condition persists, the flags will be set again. All error flags
        are "sticky" - they remain set even if the error condition goes
        away, until explicitly cleared by this command or a device reset.

        Returns:
            DeviceStatus: The device status from before it was cleared
        """
        self._write_command(_CLEAR_DEVICE_STATUS)
        data = self._read_data(2, execution_time=_TIME_STANDARD)
        return DeviceStatus((data[0] << 16) | data[1])

    @property
    def version(self) -> Tuple[int, int]:
        """Firmware version information

        Returns:
            tuple: (major_version, minor_version)
        """
        if self._firmware_version is None:
            self._write_command(_VERSION)
            data = self._read_data(1, execution_time=_TIME_STANDARD)
            # Version is packed as two bytes in one word
            self._firmware_version = ((data[0] >> 8) & 0xFF, data[0] & 0xFF)
        return self._firmware_version

    @property
    def sht_heater_polling_supported(self) -> bool:
        """True if the firmware supports polling :attr:`sht_heater_measurements`

        Older firmware does not support Get SHT Heater Measurements, and its
        Activate SHT Heater command blocks for 1.3 seconds instead of 20ms.
        """
        return self.version >= self._SHT_HEATER_POLL_FIRMWARE

    def activate_sht_heater(self) -> None:
        """Activate the SHT sensor heater to reverse humidity creep

        Heats the SHT sensor with 200mW for 1s, after which the heater switches off
        automatically. If :attr:`sht_heater_polling_supported`, poll
        :attr:`sht_heater_measurements` to find out when heating has finished.

        Wait at least 20s after this command before starting a measurement to get
        coherent temperature values.

        Raises:
            RuntimeError: If sensor is currently measuring
        """
        self._require_idle("activate SHT heater")
        execution_time = _TIME_STANDARD if self.sht_heater_polling_supported else _TIME_SHT_HEATER
        self._write_command(_ACTIVATE_SHT_HEATER, execution_time=execution_time)

    @property
    def sht_heater_measurements(self) -> Dict[str, Optional[float]]:
        """SHT sensor measurements once heating from :meth:`activate_sht_heater` is finished

        Can be polled every 50ms after activating the heater. Values are None
        until heating is finished.

        Requires firmware SEN62 >= 6.0, SEN63C >= 5.0, SEN65 >= 5.0, SEN66 >= 4.0,
        SEN68 >= 7.0 or SEN69C >= 9.0.

        Returns:
            dict: {'humidity': value or None, 'temperature': value or None}

        Raises:
            RuntimeError: If sensor is measuring or firmware does not support this command
        """
        self._require_idle("read SHT heater measurements")
        if not self.sht_heater_polling_supported:
            major, minor = self._SHT_HEATER_POLL_FIRMWARE
            raise RuntimeError(f"SHT heater measurements require firmware >= {major}.{minor}")
        return self._read_values(_SHT_HEATER, _SHT_HEATER_FIELDS)

    def raw_values(self) -> Dict[str, Optional[float]]:
        """Raw sensor values for this sensor model

        Keys present depend on the model:

        - raw_humidity: Raw humidity (%), all models
        - raw_temperature: Raw temperature (°C), all models
        - raw_voc, raw_nox: Raw VOC/NOx ticks (no scale), SEN65, SEN66, SEN68, SEN69C
        - raw_co2: Raw CO2 concentration (ppm, not interpolated, updated every 5s), SEN66

        Returns:
            dict: Raw value name to value or None if unknown

        Raises:
            RuntimeError: If sensor is not in measurement mode
        """
        if self._RAW_VALUES_COMMAND is None:
            raise NotImplementedError("Use the model-specific class, e.g. SEN66")
        self._require_measuring()
        return self._read_values(self._RAW_VALUES_COMMAND, self._RAW_VALUES_FIELDS)

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
        self._require_measuring()
        return self._read_values(_READ_NUMBER_CONCENTRATION, _NUMBER_CONCENTRATION_FIELDS)

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
        if not 0 <= time_constant <= 0xFFFF:
            raise ValueError("time_constant must be 0-65535 seconds")

        # Scale factors - these are signed int16 values
        offset_scaled = round(offset * 200)
        slope_scaled = round(slope * 10000)
        if not -32768 <= offset_scaled <= 32767 or not -32768 <= slope_scaled <= 32767:
            raise ValueError("offset or slope out of range")

        # Convert signed to unsigned for I2C transmission
        offset_scaled &= 0xFFFF
        slope_scaled &= 0xFFFF

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
        self._require_idle("set temperature acceleration")

        # Scale factors (multiply by 10 for protocol)
        data = [round(value * 10) for value in (k, p, t1, t2)]
        if not all(0 <= value <= 0xFFFF for value in data):
            raise ValueError("Temperature acceleration parameters must be 0-6553.5")
        self._write_command(_TEMP_ACCELERATION, data=data, execution_time=_TIME_STANDARD)

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


class VOCNOxMixin:
    """VOC and NOx index features (SEN65, SEN66, SEN68, SEN69C)"""

    @property
    def voc_index(self) -> Optional[float]:
        """VOC index (1.0-500.0)"""
        self.all_measurements()
        return self._measurement_data["voc_index"] if self._measurement_data else None

    @property
    def nox_index(self) -> Optional[float]:
        """NOx index (1.0-500.0)

        Unknown (None) for the first 10-11 seconds after power-on or reset.
        """
        self.all_measurements()
        return self._measurement_data["nox_index"] if self._measurement_data else None

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
        self._require_idle("set VOC state")

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
        self._require_idle("read VOC tuning")

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
        self._require_idle("set VOC tuning")

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
        self._require_idle("read NOx tuning")

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
        self._require_idle("set NOx tuning")

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


class FormaldehydeMixin:
    """Formaldehyde (HCHO) features (SEN68, SEN69C)"""

    @property
    def hcho(self) -> Optional[float]:
        """Formaldehyde concentration in ppb

        Unknown (None) for the first 60 seconds after the first measurement
        start after power-on or reset.
        """
        self.all_measurements()
        return self._measurement_data["hcho"] if self._measurement_data else None


class CO2Mixin:
    """CO2 sensor features (SEN63C, SEN66, SEN69C)"""

    @property
    def co2(self) -> Optional[float]:
        """CO2 concentration in ppm

        Unknown (None) for the first 5-6 seconds (SEN66) or 22-24 seconds
        (SEN63C, SEN69C) after starting a measurement.
        """
        self.all_measurements()
        return self._measurement_data["co2"] if self._measurement_data else None

    def force_co2_recalibration(self, target_co2_ppm: int) -> Optional[int]:
        """Perform forced CO2 recalibration (FRC)

        Forces the CO2 sensor to recalibrate to a known reference concentration.
        Operate the sensor for at least 3 minutes in an environment with a
        homogeneous and constant CO2 concentration (e.g., fresh outdoor air at
        ~420 ppm), then call stop_measurement() before calling this.

        Args:
            target_co2_ppm: Known CO2 concentration in ppm at current location

        Returns:
            int: CO2 correction applied in ppm, or None if recalibration failed

        Raises:
            RuntimeError: If sensor is currently measuring, or within the
                SEN63C/SEN69C CO2 conditioning period

        Note: This calibration is persistent across resets and power cycles.
        """
        self._require_idle("recalibrate CO2")
        self._check_co2_conditioning("recalibrate CO2")

        # Send target CO2 concentration
        self._write_command(
            _FORCE_CO2_RECALIBRATION, data=[target_co2_ppm], execution_time=_TIME_CO2_RECALIBRATION
        )

        # Read correction value
        data = self._read_data(1, execution_time=0)  # No additional wait, already waited 500ms

        # Check if recalibration failed
        if data[0] == _UNKNOWN_UINT16:
            return None

        # Calculate actual correction: correction = return_value - 0x8000
        correction = data[0] - 0x8000
        return correction

    def co2_factory_reset(self) -> None:
        """Perform a CO2 sensor factory reset

        Resets all CO2 sensor configuration stored in EEPROM and erases the forced
        recalibration (FRC) and automatic self-calibration (ASC) history, restarting
        the bypass phase. Requires SEN66 firmware >= 1.2.

        Raises:
            RuntimeError: If sensor is currently measuring, or within the
                SEN63C/SEN69C CO2 conditioning period
        """
        self._require_idle("reset the CO2 sensor")
        self._check_co2_conditioning("reset the CO2 sensor")
        self._write_command(_CO2_FACTORY_RESET, execution_time=_TIME_CO2_FACTORY_RESET)

    @property
    def co2_automatic_self_calibration(self) -> bool:
        """CO2 sensor automatic self-calibration (ASC) status

        Returns:
            bool: True if ASC is enabled, False if disabled
        """
        self._require_idle("read CO2 ASC")

        self._write_command(_CO2_AUTO_CALIB)
        data = self._read_data(1, execution_time=_TIME_STANDARD)

        # Data format: [padding_byte, status_byte] packed in one word
        # Extract status byte (LSB)
        return bool(data[0] & 0xFF)

    @co2_automatic_self_calibration.setter
    def co2_automatic_self_calibration(self, enabled: bool) -> None:
        """CO2 sensor automatic self-calibration (ASC) status

        ASC assumes the sensor is exposed to fresh air (~400 ppm) at least
        once per week. Only disable for testing under lab conditions where
        concentrations below 400 ppm are expected.

        Args:
            enabled: True to enable ASC, False to disable

        Note: Default is enabled. Setting is volatile (reset on power cycle).
        """
        self._require_idle("set CO2 ASC")
        self._check_co2_conditioning("set CO2 ASC")

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
        self._require_idle("read altitude")

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
        self._require_idle("set altitude")

        if not 0 <= altitude_m <= 3000:
            raise ValueError("Altitude must be 0-3000 meters")

        self._write_command(_SENSOR_ALTITUDE, data=[altitude_m], execution_time=_TIME_STANDARD)


class SEN62(SEN6x):
    """Driver for SEN62 sensor - measures PM, RH, and Temperature"""

    _MEASUREMENT_COMMAND = _SEN62_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS
    _RAW_VALUES_COMMAND = _SEN62_SEN63C_READ_RAW_VALUES
    _RAW_VALUES_FIELDS = _RAW_RHT_FIELDS
    _SHT_HEATER_POLL_FIRMWARE = (6, 0)


class SEN63C(CO2Mixin, SEN6x):
    """Driver for SEN63C sensor - measures PM, CO2, RH, and Temperature

    Note: The CO2 sensor requires a 24-second conditioning period after starting
    a measurement. During this time, CO2 values will be reported as unknown (None).
    """

    _MEASUREMENT_COMMAND = _SEN63C_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS + _CO2_INT16_FIELDS
    _RAW_VALUES_COMMAND = _SEN62_SEN63C_READ_RAW_VALUES
    _RAW_VALUES_FIELDS = _RAW_RHT_FIELDS
    _SHT_HEATER_POLL_FIRMWARE = (5, 0)
    _CO2_CONDITIONING_TIME = 24


class SEN65(VOCNOxMixin, SEN6x):
    """Driver for SEN65 sensor - measures PM, VOC, NOx, RH, and Temperature"""

    _MEASUREMENT_COMMAND = _SEN65_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS + _VOC_NOX_FIELDS
    _RAW_VALUES_COMMAND = _SEN65_SEN68_SEN69C_READ_RAW_VALUES
    _RAW_VALUES_FIELDS = _RAW_RHT_FIELDS + _RAW_VOC_NOX_FIELDS
    _SHT_HEATER_POLL_FIRMWARE = (5, 0)


class SEN66(VOCNOxMixin, CO2Mixin, SEN6x):
    """Driver for SEN66 sensor - measures PM, VOC, NOx, CO2, RH, and Temperature"""

    _MEASUREMENT_COMMAND = _SEN66_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS + _VOC_NOX_FIELDS + _CO2_UINT16_FIELDS
    _RAW_VALUES_COMMAND = _SEN66_READ_RAW_VALUES
    _RAW_VALUES_FIELDS = _RAW_RHT_FIELDS + _RAW_VOC_NOX_FIELDS + _RAW_CO2_FIELDS
    _SHT_HEATER_POLL_FIRMWARE = (4, 0)


class SEN68(VOCNOxMixin, FormaldehydeMixin, SEN6x):
    """Driver for SEN68 sensor - measures PM, VOC, NOx, HCHO, RH, and Temperature"""

    _MEASUREMENT_COMMAND = _SEN68_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS + _VOC_NOX_FIELDS + _HCHO_FIELDS
    _RAW_VALUES_COMMAND = _SEN65_SEN68_SEN69C_READ_RAW_VALUES
    _RAW_VALUES_FIELDS = _RAW_RHT_FIELDS + _RAW_VOC_NOX_FIELDS
    _SHT_HEATER_POLL_FIRMWARE = (7, 0)


class SEN69C(VOCNOxMixin, FormaldehydeMixin, CO2Mixin, SEN6x):
    """Driver for SEN69C sensor - measures PM, VOC, NOx, HCHO, CO2, RH, and Temperature

    Note: The CO2 sensor requires a 24-second conditioning period after starting
    a measurement. During this time, CO2 values will be reported as unknown (None).
    """

    _MEASUREMENT_COMMAND = _SEN69C_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS + _VOC_NOX_FIELDS + _HCHO_FIELDS + _CO2_INT16_FIELDS
    _RAW_VALUES_COMMAND = _SEN65_SEN68_SEN69C_READ_RAW_VALUES
    _RAW_VALUES_FIELDS = _RAW_RHT_FIELDS + _RAW_VOC_NOX_FIELDS
    _SHT_HEATER_POLL_FIRMWARE = (9, 0)
    _CO2_CONDITIONING_TIME = 24


class SEN60(_SEN6xBase):
    """Driver for SEN60 sensor (discontinued) - measures PM mass and number concentration

    The SEN60 uses a different I2C address (0x6C) and command set from the rest
    of the SEN6x family. It has no RH/T, gas or CO2 sensors, and does not support
    product name, firmware version or the temperature compensation commands.

    Args:
        i2c: The I2C bus the sensor is connected to
        address: The I2C address of the sensor (default: 0x6C)
        startup_delay: Seconds to wait for the sensor to start up (default: 1.0).
            Can be set to 0 if the sensor has already been powered for a while.
    """

    _START_COMMAND = _SEN60_START_MEASUREMENT
    _STOP_COMMAND = _SEN60_STOP_MEASUREMENT
    _DATA_READY_COMMAND = _SEN60_DATA_READY
    _DATA_READY_MASK = 0x07FF  # Bits 10..0, non-zero when data is ready
    _FAN_CLEANING_COMMAND = _SEN60_FAN_CLEANING
    _TIME_COMMAND = _TIME_SEN60_COMMAND
    _TIME_START = _TIME_SEN60_COMMAND
    _TIME_STOP = _TIME_SEN60_STOP_MEASUREMENT
    _MEASUREMENT_COMMAND = _SEN60_READ_MEASUREMENT
    _MEASUREMENT_FIELDS = _PM_RHT_FIELDS[:4] + _NUMBER_CONCENTRATION_FIELDS

    def __init__(
        self,
        i2c: I2C,
        address: int = SEN60_I2C_ADDRESS,
        startup_delay: float = _SENSOR_STARTUP_TIME,
    ) -> None:
        super().__init__(i2c, address, startup_delay)

    def reset(self) -> None:
        """Reset the sensor

        Has the same effect as a power cycle. Can be used in idle or measurement
        mode; the sensor returns to idle mode.
        """
        self._write_command(_SEN60_RESET, execution_time=_TIME_SEN60_COMMAND)
        self._measurement_started = False
        self._serial_number = None
        self._measurement_data = None
        # Wait for sensor to restart
        time.sleep(_SENSOR_STARTUP_TIME)

    @property
    def serial_number(self) -> str:
        """The sensor serial number as a 12-digit hexadecimal string

        Can only be read from the sensor in idle mode, the value is cached afterwards.
        """
        if self._serial_number is None:
            self._require_idle("read serial number")
            self._write_command(_SEN60_SERIAL_NUMBER, execution_time=0)
            data = self._read_data(3, execution_time=_TIME_SEN60_COMMAND)
            self._serial_number = "".join(f"{word:04X}" for word in data)
        return self._serial_number

    @property
    def device_status(self) -> DeviceStatus:
        """The device status register

        The SEN60 has a 16-bit status register with only the fan speed warning
        (bit 1) and fan error (bit 4). These are mapped onto the same
        :class:`DeviceStatus` flags as the other SEN6x models.

        Note: Error flags can only be cleared by :meth:`reset` or a power cycle.

        Returns:
            DeviceStatus: Object containing parsed status information
        """
        self._write_command(_SEN60_DEVICE_STATUS, execution_time=0)
        data = self._read_data(1, execution_time=_TIME_SEN60_COMMAND)
        status = data[0] & (1 << _STATUS_FAN_ERROR)
        if data[0] & (1 << _SEN60_STATUS_SPEED_WARNING):
            status |= 1 << _STATUS_SPEED_WARNING
        return DeviceStatus(status)

    def all_measurements(self) -> Dict[str, Optional[float]]:
        """All measurement values from SEN60

        The SEN60 only returns each measurement once and NACKs further reads until
        new data is available. In that case the previously read values are returned.

        Returns:
            dict:
                - pm1_0, pm2_5, pm4_0, pm10: Mass concentration (µg/m³)
                - nc_pm0_5, nc_pm1_0, nc_pm2_5, nc_pm4_0, nc_pm10: Number
                  concentration (particles/cm³)

        Raises:
            RuntimeError: If sensor is not in measurement mode
            OSError: If no measurement has been read yet and none is available
        """
        try:
            return super().all_measurements()
        except OSError:
            if self._measurement_data is None:
                raise
            return self._measurement_data

    def number_concentration(self) -> Dict[str, Optional[float]]:
        """Particle number concentration values

        Returns:
            dict: Dictionary containing number concentrations (particles/cm³):
                nc_pm0_5, nc_pm1_0, nc_pm2_5, nc_pm4_0 and nc_pm10
        """
        data = self.all_measurements()
        return {name: data[name] for name, _, _ in _NUMBER_CONCENTRATION_FIELDS}


# Product name prefix to driver class, for models at SEN6X_I2C_ADDRESS
_SEN6X_MODELS = (
    ("SEN62", SEN62),
    ("SEN63C", SEN63C),
    ("SEN65", SEN65),
    ("SEN66", SEN66),
    ("SEN68", SEN68),
    ("SEN69C", SEN69C),
)


def create_sensor(
    i2c: I2C, address: Optional[int] = None, startup_delay: float = _SENSOR_STARTUP_TIME
) -> Union[SEN6x, SEN60]:
    """Detect which SEN6x model is connected and create the matching driver

    Reads the product name of a sensor at 0x6B to pick between :class:`SEN62`,
    :class:`SEN63C`, :class:`SEN65`, :class:`SEN66`, :class:`SEN68` and
    :class:`SEN69C`. If no sensor responds there, a :class:`SEN60` at 0x6C is used.

    .. code-block:: python

        sensor = adafruit_sen6x.create_sensor(board.I2C())
        print(type(sensor).__name__)  # e.g. SEN66

    Args:
        i2c: The I2C bus the sensor is connected to
        address: Only look for a sensor at this address. 0x6C (SEN60_I2C_ADDRESS)
            creates a :class:`SEN60`, any other address is treated as a SEN6x.
            Default None tries 0x6B, then 0x6C.
        startup_delay: Seconds to wait for the sensor to start up (default: 1.0).
            Can be set to 0 if the sensor has already been powered for a while.

    Raises:
        ValueError: If no sensor is found
        RuntimeError: If the product name is not a known SEN6x model
    """
    time.sleep(startup_delay)
    if address != SEN60_I2C_ADDRESS:
        sen6x_address = SEN6X_I2C_ADDRESS if address is None else address
        try:
            probe = SEN6x(i2c, sen6x_address, startup_delay=0)
        except ValueError:  # No device at this address
            if address is not None:
                raise
        else:
            sensor = probe.model_class(i2c, sen6x_address, startup_delay=0)
            sensor._product_name = probe.product_name
            sensor._model_class = probe.model_class
            return sensor
    return SEN60(i2c, SEN60_I2C_ADDRESS if address is None else address, startup_delay=0)
