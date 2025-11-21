# BNO08X Micropython Driver by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# Additional functions written by dobodu also adapted
#

"""
`bno08x`
================================================================================

Helper library for the CEVA  Hillcrest Laboratories BNO08x IMUs

* Author(s): Bryan Siepert
* Author(s): dobodu
* Author(s): bradcar

Implementation Notes
--------------------

**Hardware:**

* bno086

**Software and Dependencies:**

* MicroPython

TODO: updating sensor values more asychronously
TODO: add 3-tuple, 4-tuple, 5-tuple returns, also with .meta and .full returns with acc & ts

TODO: debug soft reset in SPI

TODO: BRC test i2c with Reset & Interrupt pins (no wake?)
TODO: BRC I2c add quick fail and good error message if no i2c devices found
TODO: BRC add protection to ensure only pin objects, not numbers are passed in (spi.py, i2c.py, and uart)
TODO: BRC Euler/quaternion implementation
TODO: BRC add TARE
# TODO:
# Default Reports Frequencies (Hz) - does this code have report frequencies right?
# Calibrated Acceleration (m/s^2)
# Euler Angles (in degrees?)
# CALIBRATION

#
Delay
1. When the timebase reference report is provided with individual sensor report
   it will likely have a delay of zero.
2. In cases where sensor reports are concatenated (due to delays in processing),
   the delay field may be populated, then delay and the timebase reference
   are used to calculate the sensor sample's actual timestamp.

"""

__version__ = "0.1"
__repo__ = "https://github.com/bradcar/bno08x_i2c_spi_MicroPython"

from math import asin, atan2, degrees
from struct import pack_into, unpack_from

import uctypes
from collections import namedtuple
from machine import Pin
from micropython import const
from utime import ticks_us, ticks_ms, ticks_diff, sleep_ms, sleep_us

# import support files
from debug import channels, reports

# Commands
BNO_CHANNEL_SHTP_COMMAND = const(0)
BNO_CHANNEL_EXE = const(1)
_BNO_CHANNEL_CONTROL = const(2)
_BNO_CHANNEL_INPUT_SENSOR_REPORTS = const(3)
_BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = const(4)
_BNO_CHANNEL_GYRO_ROTATION_VECTOR = const(5)

_GET_FEATURE_REQUEST = const(0xFE)
_SET_FEATURE_COMMAND = const(0xFD)
_GET_FEATURE_RESPONSE = const(0xFC)
_BASE_TIMESTAMP = const(0xFB)
_TIMESTAMP_REBASE = const(0xFA)
_SHTP_REPORT_PRODUCT_ID_REQUEST = const(0xF9)
_SHTP_REPORT_PRODUCT_ID_RESPONSE = const(0xF8)
_FRS_WRITE_REQUEST = const(0xF7)
_FRS_WRITE_DATA = const(0xF6)
_FRS_WRITE_RESPONSE = const(0xF5)
_FRS_READ_REQUEST = const(0xF4)
_FRS_READ_RESPONSE = const(0xF3)
_COMMAND_REQUEST = const(0xF2)
_COMMAND_RESPONSE = const(0xF1)

# DCD/ ME Calibration commands and sub-commands
_SAVE_DCD = const(0x6)
_ME_CALIBRATE = const(0x7)
_ME_CAL_CONFIG = const(0x00)
_ME_GET_CAL = const(0x01)

# Reports Summary depending on BNO device
BNO_REPORT_ACCELEROMETER = const(0x01)  # Calibrated Acceleration (m/s2)
BNO_REPORT_GYROSCOPE = const(0x02)  # Calibrated gyroscope (rad/s).
BNO_REPORT_MAGNETOMETER = const(0x03)  # Magnetic field calibrated (in µTesla).
BNO_REPORT_LINEAR_ACCELERATION = const(0x04)  # Linear acceleration (m/s2) with gravity removed
BNO_REPORT_ROTATION_VECTOR = const(0x05)  # Rotation Vector
BNO_REPORT_GRAVITY = const(0x06)  # Gravity Vector (m/s2). Vector direction of gravity
BNO_REPORT_UNCALIBRATED_GYROSCOPE = const(0x07)
BNO_REPORT_GAME_ROTATION_VECTOR = const(0x08)  # Game Rotation Vector
BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = const(0x09)
BNO_REPORT_PRESSURE = const(0x0A)
BNO_REPORT_AMBIENT_LIGHT = const(0x0B)
BNO_REPORT_HUMIDITY = const(0x0C)
BNO_REPORT_PROXIMITY = const(0x0D)
BNO_REPORT_TEMPERATURE = const(0x0E)
BNO_REPORT_UNCALIBRATED_MAGNETOMETER = const(0x0F)
BNO_REPORT_TAP_DETECTOR = const(0x10)
BNO_REPORT_STEP_COUNTER = const(0x11)
BNO_REPORT_SIGNIFICANT_MOTION = const(0x12)
BNO_REPORT_STABILITY_CLASSIFIER = const(0x13)
BNO_REPORT_RAW_ACCELEROMETER = const(0x14)
BNO_REPORT_RAW_GYROSCOPE = const(0x15)
BNO_REPORT_RAW_MAGNETOMETER = const(0x16)
BNO_REPORT_SAR = const(0x17)
BNO_REPORT_STEP_DETECTOR = const(0x18)
BNO_REPORT_SHAKE_DETECTOR = const(0x19)
BNO_REPORT_FLIP_DETECTOR = const(0x1A)
BNO_REPORT_PICKUP_DETECTOR = const(0x1B)
BNO_REPORT_STABILITY_DETECTOR = const(0x1C)
BNO_REPORT_ACTIVITY_CLASSIFIER = const(0x1E)
BNO_REPORT_SLEEP_DETECTOR = const(0x1F)
BNO_REPORT_TILT_DETECTOR = const(0x20)
BNO_REPORT_POCKET_DETECTOR = const(0x21)
BNO_REPORT_CIRCLE_DETECTOR = const(0x22)
BNO_REPORT_HEART_RATE_MONITOR = const(0x23)
BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR = const(0x28)
BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR = const(0x29)
BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = const(0x2A)

_REPORTS_DICTIONARY = {
    0x01: "ACCELEROMETER",
    0x02: "GYROSCOPE",
    0x03: "MAGNETIC_FIELD",
    0x04: "LINEAR_ACCELERATION",
    0x05: "ROTATION_VECTOR",
    0x06: "GRAVITY",
    0x07: "UNCALIBRATED_GYROSCOPE",
    0x08: "GAME_ROTATION_VECTOR",
    0x09: "GEOMAGNETIC_ROTATION_VECTOR",
    0x0A: "PRESSURE",
    0x0B: "AMBIENT LIGHT",
    0x0C: "HUMIDITY",
    0x0D: "PROXIMITY",
    0x0E: "TEMPERATURE",
    0x0F: "UNCALIBRATED_MAGNETIC_FIELD",
    0x10: "TAP_DETECTOR",
    0x11: "STEP_COUNTER",
    0x12: "SIGNIFICANT_MOTION",
    0x13: "STABILITY_CLASSIFIER",
    0x14: "RAW_ACCELEROMETER",
    0x15: "RAW_GYROSCOPE",
    0x16: "RAW_MAGNETOMETER",
    0x17: "SAR",
    0x18: "STEP_DETECTOR",
    0x19: "SHAKE_DETECTOR",
    0x1A: "FLIP_DETECTOR",
    0x1B: "PICKUP_DETECTOR",
    0x1C: "STABILITY_DETECTOR",
    0x1E: "PERSONAL_ACTIVITY_CLASSIFIER",
    0x1F: "SLEEP_DETECTOR",
    0x20: "TILT_DETECTOR",
    0x21: "POCKET_DETECTOR",
    0x22: "CIRCLE_DETECTOR",
    0x23: "HR MONITOR",
    0x28: "ARVR_STABILIZED_ROTATION_VECTOR",
    0x29: "ARVR_STABILIZED_GAME_ROTATION_VECTOR",
    0x2A: "GYRO INTEGRATED ROTATION VECTOR",
    0xF1: "COMMAND_RESPONSE",
    0xF2: "COMMAND_REQUEST",
    0xF3: "FRS_READ_RESPONSE",
    0xF4: "FRS_READ_REQUEST",
    0xF5: "FRS_WRITE_RESPONSE",
    0xF6: "FRS_WRITE_DATA",
    0xF7: "FRS_WRITE_REQUEST",
    0xF8: "PRODUCT_ID_RESPONSE",
    0xF9: "PRODUCT_ID_REQUEST",
    0xFA: "TIMESTAMP_REBASE",
    0xFB: "BASE_TIMESTAMP",
    0xFC: "GET_FEATURE_RESPONSE",
    0xFD: "SET_FEATURE_COMMAND",
    0xFE: "GET_FEATURE_REQUEST",
}

_DEFAULT_REPORT_INTERVAL = const(50_000)  # 50,000us = 50ms, 20 MHz
_QUAT_READ_TIMEOUT = 0.5  # timeout in seconds
_PACKET_READ_TIMEOUT = 2.0  # timeout in seconds
_FEATURE_ENABLE_TIMEOUT = 2.0  # timeout in seconds
_DEFAULT_TIMEOUT = 2.0  # timeout in seconds
_CALIBRATION_TIMEOUT = 5.0  # 10 sec
_TIMEOUT_SHORT_MS = 100  # short sensor knows ME status, just needs to package and send (few ms)
_BNO08X_CMD_RESET = const(0x01)
_QUAT_Q_POINT = const(14)  # TODO dobodu sets to 0x05 ???
_BNO_HEADER_LEN = const(4)

# Status Constants
_COMMAND_STATUS_SUCCESS = 0

# Report Frequencies in Hertz
DEFAULT_REPORT_FREQ = {
    BNO_REPORT_ACCELEROMETER: 20,
    BNO_REPORT_GYROSCOPE: 20,
    BNO_REPORT_MAGNETOMETER: 20,
    BNO_REPORT_LINEAR_ACCELERATION: 20,
    BNO_REPORT_ROTATION_VECTOR: 10,
    BNO_REPORT_GRAVITY: 10,
    BNO_REPORT_GAME_ROTATION_VECTOR: 10,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: 10,
    BNO_REPORT_PRESSURE: 2,
    BNO_REPORT_AMBIENT_LIGHT: 10,
    BNO_REPORT_HUMIDITY: 2,
    BNO_REPORT_PROXIMITY: 10,
    BNO_REPORT_TEMPERATURE: 2,
    BNO_REPORT_STEP_COUNTER: 5,
    BNO_REPORT_SHAKE_DETECTOR: 20,
    BNO_REPORT_STABILITY_CLASSIFIER: 2,
    BNO_REPORT_ACTIVITY_CLASSIFIER: 2,
    BNO_REPORT_RAW_ACCELEROMETER: 20,
    BNO_REPORT_RAW_GYROSCOPE: 20,
    BNO_REPORT_RAW_MAGNETOMETER: 20,
    BNO_REPORT_UNCALIBRATED_GYROSCOPE: 20,
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER: 20,
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: 10,
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: 10,
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: 10,
}

_Q_POINT_14_SCALAR = 2 ** (14 * -1)
_Q_POINT_12_SCALAR = 2 ** (12 * -1)
# _Q_POINT_10_SCALAR = 2 ** (10 * -1)
_Q_POINT_9_SCALAR = 2 ** (9 * -1)
_Q_POINT_8_SCALAR = 2 ** (8 * -1)
_Q_POINT_4_SCALAR = 2 ** (4 * -1)

_GYRO_SCALAR = _Q_POINT_9_SCALAR
_ACCEL_SCALAR = _Q_POINT_8_SCALAR
_QUAT_SCALAR = _Q_POINT_14_SCALAR
_GEO_QUAT_SCALAR = _Q_POINT_12_SCALAR
_MAG_SCALAR = _Q_POINT_4_SCALAR

_REPORT_LENGTHS = {
    _SHTP_REPORT_PRODUCT_ID_RESPONSE: 16,
    _GET_FEATURE_RESPONSE: 17,
    _COMMAND_RESPONSE: 16,
    _BASE_TIMESTAMP: 5,
    _TIMESTAMP_REBASE: 5,
}
# these raw reports require their counterpart to be enabled
_RAW_REPORTS = {
    BNO_REPORT_RAW_ACCELEROMETER: BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE: BNO_REPORT_GYROSCOPE,
    BNO_REPORT_UNCALIBRATED_GYROSCOPE: BNO_REPORT_GYROSCOPE,  # For testing
    BNO_REPORT_RAW_MAGNETOMETER: BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER: BNO_REPORT_MAGNETOMETER,  # For testing
}

# The last cause of the processor reset.
_RESET_CAUSE_STRING = [
    "Not Applicable",
    "Power On Reset",
    "Internal System Reset",
    "Watchdog Timeout",
    "External Reset",
    "Other",
]

# Available sensor reports
_AVAIL_SENSOR_REPORTS = {
    BNO_REPORT_ACCELEROMETER: (_Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_GYROSCOPE: (_Q_POINT_9_SCALAR, 3, 10),
    BNO_REPORT_MAGNETOMETER: (_Q_POINT_4_SCALAR, 3, 10),
    BNO_REPORT_LINEAR_ACCELERATION: (_Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 14),
    BNO_REPORT_GRAVITY: (_Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (_Q_POINT_12_SCALAR, 4, 14),
    BNO_REPORT_PRESSURE: (1, 1, 8),
    BNO_REPORT_AMBIENT_LIGHT: (1, 1, 8),
    BNO_REPORT_HUMIDITY: (1, 1, 6),
    BNO_REPORT_PROXIMITY: (1, 1, 6),
    BNO_REPORT_TEMPERATURE: (1, 1, 6),
    BNO_REPORT_STEP_COUNTER: (1, 1, 12),
    BNO_REPORT_SHAKE_DETECTOR: (1, 1, 6),
    BNO_REPORT_STABILITY_CLASSIFIER: (1, 1, 6),
    BNO_REPORT_ACTIVITY_CLASSIFIER: (1, 1, 16),
    BNO_REPORT_RAW_ACCELEROMETER: (1, 3, 16),
    BNO_REPORT_RAW_GYROSCOPE: (1, 3, 16),
    BNO_REPORT_RAW_MAGNETOMETER: (1, 3, 16),
    BNO_REPORT_UNCALIBRATED_GYROSCOPE: (_Q_POINT_9_SCALAR, 3, 10),  # For testing
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER: (_Q_POINT_4_SCALAR, 3, 10),  # For testing
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 5, 14),  # For testing
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),  # For testing
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),  # For testing
}

# C-style layout for standard BNO08x sensor reports 16-bit
# Offsets follow the BNO08x standard header structure
_SENSOR_REPORT_LAYOUT = {
    "report_id": 0 | uctypes.UINT8,
    "status": 1 | uctypes.UINT8,

    # byte2 contains accuracy+delay high bits, byte 3 has delay low bits
    "byte2": 2 | uctypes.UINT8,
    "byte3": 3 | uctypes.UINT8,

    "v1": 4 | uctypes.INT16,
    "v2": 6 | uctypes.INT16,
    "v3": 8 | uctypes.INT16,  # valid valid for 3-tuple reports
    "v4": 10 | uctypes.INT16,  # only valid for quaternion reports
    "e1": 12 | uctypes.INT16,  # only valid for ARVR rotation report: quaternion + estimaate
}

_INITIAL_REPORTS = {
    BNO_REPORT_ACTIVITY_CLASSIFIER: {
        "Tilting": -1,
        "most_likely": "Unknown",
        "OnStairs": -1,
        "On-Foot": -1,
        "Other": -1,
        "On-Bicycle": -1,
        "Still": -1,
        "Walking": -1,
        "Unknown": -1,
        "Running": -1,
        "In-Vehicle": -1,
    },
    BNO_REPORT_STABILITY_CLASSIFIER: "Unknown",
    BNO_REPORT_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0),
    BNO_REPORT_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0),
    # Gyro is a 5 tuple, Celsius float and int timestamp for last two entry
    BNO_REPORT_RAW_GYROSCOPE: (0, 0, 0, 0.0, 0, 0, 0),
    # Acc & Mag are 4-tuple, int timestamp for last entry
    BNO_REPORT_RAW_ACCELEROMETER: (0, 0, 0, 0, 0, 0),
    BNO_REPORT_RAW_MAGNETOMETER: (0, 0, 0, 0, 0, 0),
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0.0, 0, 0),
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0),
    BNO_REPORT_STEP_COUNTER: 0,
}

# Activity classifier Initialization
ACTIVITIES = ["Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still", "Tilting", "Walking", "Running", "OnStairs", ]
_ENABLED_ACTIVITIES = 0x1FF  # Enable 9  activities: 1 bit set for each of 8 activities and 1 Unknown

DATA_BUFFER_SIZE = const(512)  # data buffer size. obviously eats ram
PacketHeader = namedtuple(
    "PacketHeader",
    [
        "channel_number",
        "sequence_number",
        "data_length",
        "packet_byte_count",
    ],
)

REPORT_ACCURACY_STATUS = [
    "Accuracy Unreliable",
    "Low Accuracy",
    "Medium Accuracy",
    "High Accuracy",
]


class PacketError(Exception):
    """Raised when the packet could not be parsed"""
    pass


# Elapsed seconds, pass in tick_ms
def _elapsed_sec(start_time):
    """
    Elapsed time between now - start_time.  You pass in start_time = ticks_ms()
    Returns float in seconds
    """
    return ticks_diff(ticks_ms(), start_time) / 1000.0


############ REPORT PARSING ###########################
def _parse_sensor_report_data(report_bytes: bytearray) -> tuple[tuple, int, int]:
    """
    uctypes-based parser for BNO08x sensor reports, sensor data starts at offset=4
    Parses 3-tuple vectors, 4-tuple quaternions, and 5-tuple ARVR rotation.
    """
    s = uctypes.struct(uctypes.addressof(report_bytes), _SENSOR_REPORT_LAYOUT, uctypes.LITTLE_ENDIAN)
    report_id = s.report_id
    scalar, count, _ = _AVAIL_SENSOR_REPORTS[report_id]

    # Extract accuracy from byte2 low bits
    accuracy = s.byte2 & 0x03

    # Extract delay (14 bits)
    delay_upper = (s.byte2 >> 2) & 0x3F
    delay_raw = (delay_upper << 8) | s.byte3
    delay_us = delay_raw * 100

    # Read & scale sensor fields using uctypes INT16 fields
    if count == 3:
        scaled_data = (
            s.v1 * scalar,
            s.v2 * scalar,
            s.v3 * scalar,
        )
    elif count == 4:
        scaled_data = (
            s.v1 * scalar,
            s.v2 * scalar,
            s.v3 * scalar,
            s.v4 * scalar,
        )
    elif count == 5:
        scaled_data = (
            s.v1 * scalar,
            s.v2 * scalar,
            s.v3 * scalar,
            s.v4 * scalar,
            s.e1,
        )
    else:
        raise ValueError(f"Unexpected tuple length {count}")

    return scaled_data, accuracy, delay_us


def _parse_sensor_id(buffer: bytearray) -> tuple[int, ...]:
    """Parse the fields of a product id report"""
    if not buffer[0] == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
        raise AttributeError(f"Wrong report id for sensor id: {hex(buffer[0])}")
    reset_cause = unpack_from("<B", buffer, 1)[0]
    sw_major = unpack_from("<B", buffer, 2)[0]
    sw_minor = unpack_from("<B", buffer, 3)[0]
    sw_part_number = unpack_from("<I", buffer, 4)[0]
    sw_build_number = unpack_from("<I", buffer, 8)[0]
    sw_patch = unpack_from("<H", buffer, 12)[0]
    return reset_cause, sw_part_number, sw_major, sw_minor, sw_patch, sw_build_number


############ COMMAND PARSING ###########################
def _parse_command_response(report_bytes: bytearray):
    # CMD response report:
    # 0 Report ID = 0xF1
    # 1 Sequence number
    # 2 Command
    # 3 Command sequence number
    # 4 Response sequence number
    # 5 R0-10 A set of response values. The interpretation of these values is specific
    # to the response for each command.
    report_body = unpack_from("<BBBBB", report_bytes)
    response_values = unpack_from("<BBBBBBBBBBB", report_bytes, 5)
    return report_body, response_values


def _insert_command_request_report(
        command: int,
        buffer: bytearray,
        next_sequence_number: int,
        command_params=None,
) -> None:
    if command_params and len(command_params) > 9:
        raise AttributeError(f"Command request report max 9 arguments: {len(command_params)} given")
    buffer[0] = _COMMAND_REQUEST
    buffer[1] = next_sequence_number
    buffer[2] = command
    for _i in range(3, 12):
        buffer[_i] = 0
    if command_params is None:
        return

    for idx, param in enumerate(command_params):
        buffer[3 + idx] = param


class Packet:
    """A class representing a Hillcrest Laboratory Sensor Hub Transport packet"""

    def __init__(self, packet_bytes: bytearray) -> None:
        self.header = self.header_from_buffer(packet_bytes)
        data_end_index = self.header.data_length + _BNO_HEADER_LEN
        self.data = packet_bytes[_BNO_HEADER_LEN:data_end_index]

    def __str__(self) -> str:
        length = self.header.packet_byte_count
        outstr = "\n\t\t********** Packet *************\n"
        outstr += "DBG::\t\tHeader:\n"
        outstr += f"DBG::\t\t Data Len: {self.header.data_length}\n"
        outstr += f"DBG::\t\t Channel: {channels[self.channel_number]} ({self.channel_number})\n"
        if self.channel_number in {_BNO_CHANNEL_CONTROL, _BNO_CHANNEL_INPUT_SENSOR_REPORTS, }:
            if self.report_id in reports:
                outstr += f"DBG::\t\t Report Type: {reports[self.report_id]} ({hex(self.report_id)})\n"
            else:
                outstr += f"DBG::\t\t \t** UNKNOWN Report Type **: {hex(self.report_id)}\n"
            if self.report_id == 0xFC and len(self.data) >= 6 and self.data[1] in reports:
                outstr += f"DBG::\t\t Enabled Feature: {reports[self.data[1]]} ({hex(self.data[1])})\n"
                outstr += f"DBG::\t\t Sequence number: {self.header.sequence_number}\n"
        outstr += "\nDBG::\t\tData:"

        for idx, packet_byte in enumerate(self.data[:length]):
            packet_index = idx + 4
            if (packet_index % 4) == 0:
                outstr += f"\nDBG::\t\t[0x{packet_index:02X}] "
            outstr += f"0x{packet_byte:02X} "
        outstr += "\n\t\t*******************************\n"
        return outstr

    @property
    def report_id(self) -> int:
        """The Packet's Report ID"""
        return self.data[0]

    @property
    def channel_number(self) -> int:
        """The packet channel"""
        return self.header.channel_number

    @classmethod
    def header_from_buffer(cls, packet_bytes: bytearray) -> PacketHeader:
        """Creates a `PacketHeader` object from a given buffer"""        
        header_data = unpack_from("<HBB", packet_bytes, 0)
        packet_byte_count = header_data[0] & ~0x8000
        channel_number = header_data[1]
        sequence_number = header_data[2]
        
        data_length = max(0, packet_byte_count - 4)
        header = PacketHeader(channel_number, sequence_number, data_length, packet_byte_count)
        return header

    @classmethod
    def is_error(cls, header: PacketHeader) -> bool:
        """Returns True if the header is an error condition"""
        if header.channel_number > 5:
            return True
        if header.packet_byte_count == 0xFFFF and header.sequence_number == 0xFF:
            return True
        return False


class SensorReading3:
    """3-element reading with optional metadata."""
    __slots__ = ("v1", "v2", "v3", "accuracy", "timestamp_us")

    def __init__(self, v1, v2, v3, accuracy, timestamp_us):
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.accuracy = accuracy
        self.timestamp_us = timestamp_us

    def __iter__(self):
        yield self.v1
        yield self.v2
        yield self.v3

    @property
    def meta(self):
        return self.accuracy, self.timestamp_us

    @property
    def full(self):
        return self.v1, self.v2, self.v3, self.accuracy, self.timestamp_us

    def __repr__(self):
        return (
            f"Sensor 3-tuple (v1={self.v1}, v2={self.v2}, v3={self.v3}, "
            f"accuracy={self.accuracy}, timestamp_us={self.timestamp_us})"
        )

class SensorReading4:
    """4-element reading with optional metadata."""
    __slots__ = ("v1", "v2", "v3", "v4", "accuracy", "timestamp_us")

    def __init__(self, v1, v2, v3, v4, accuracy, timestamp_us):
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.v4 = v4
        self.accuracy = accuracy
        self.timestamp_us = timestamp_us

    def __iter__(self):
        yield self.v1
        yield self.v2
        yield self.v3
        yield self.v4

    @property
    def meta(self):
        return self.accuracy, self.timestamp_us

    @property
    def full(self):
        return self.v1, self.v2, self.v3, self.v4, self.accuracy, self.timestamp_us

    def __repr__(self):
        return (
            f"Sensor 4-tuple (v1={self.v1}, v2={self.v2}, v3={self.v3}, v4={self.v4}, "
            f"accuracy={self.accuracy}, timestamp_us={self.timestamp_us})"
        )

# class SensorReading5:
#     """5-element reading with optional metadata."""
#     __slots__ = ("v1", "v2", "v3", "v4", "e1", "accuracy", "timestamp_us")
#
#     def __init__(self, v1, v2, v3, v4, e1, accuracy, timestamp_us):
#         self.v1 = v1
#         self.v2 = v2
#         self.v3 = v3
#         self.v4 = v4
#         self.e1 = e1
#         self.accuracy = accuracy
#         self.timestamp_us = timestamp_us
#
#     def __iter__(self):
#         yield self.v1
#         yield self.v2
#         yield self.v3
#         yield self.v4
#         yield self.e1
#
#
#     @property
#     def meta(self):
#         return self.accuracy, self.timestamp_us
#
#     @property
#     def full(self):
#         return self.v1, self.v2, self.v3, self.v4, self.e1, self.accuracy, self.timestamp_us
#
#     def __repr__(self):
#         return (
#             f"Sensor 5-tuple(v1={self.v1}, v2={self.v2}, v3={self.v3}, v4={self.v4}, e1={self.e1},"
#             f"accuracy={self.accuracy}, timestamp_us={self.timestamp_us})"
#         )

class BNO08X:
    """Library for the BNO08x IMUs from Hillcrest Laboratories

        Main flow:
        * _process_available_packets
        * _handle_packet
        * _process_report
        * _parse_sensor_report_data

    :param

    """

    def __init__(self, _interface, reset_pin=None, int_pin=None, cs_pin=None, wake_pin=None, debug=False) -> None:

        self._debug = debug
        self._reset_pin = reset_pin
        self._int_pin = int_pin
        self._wake_pin = wake_pin
        self._cs_pin = cs_pin
        self._dbg("********** __init__ on {_interface} Interface *************\n")
        self._data_buffer: bytearray = bytearray(DATA_BUFFER_SIZE)
        self._data_buffer_memoryview = memoryview(self._data_buffer)
        self._command_buffer: bytearray = bytearray(12)
        self._packet_slices = []
        self.last_interrupt_us = 0
        self.prev_interrupt_us = 0
        self._last_base_timestamp_us = 0

        # track sequence numbers one per channel, one per direction
        # RX(inbound, last seen/expected) and TX(outbound) sequence numbers
        self._rx_sequence_number: list[int] = [0, 0, 0, 0, 0, 0]
        self._tx_sequence_number: list[int] = [0, 0, 0, 0, 0, 0]

        self._two_ended_sequence_numbers: dict[int, int] = {}
        self._dcd_saved_at: float = -1
        self._me_calibration_started_at: float = -1.0
        self._calibration_complete = False
        self._magnetometer_accuracy = 0
        self._wait_for_initialize = True
        self._init_complete = False
        self._id_read = False
        self._quaternion_euler_vector = BNO_REPORT_GAME_ROTATION_VECTOR  # default can change with set_quaternion_euler
        # dictionary of most recent values from each enabled sensor report
        self._report_values = {}
        # dictionary of reports received but not yet read by user
        self._unread_report_count = {}
        self._report_periods_dictionary_us = {}

        self.reset_sensor()
        self._dbg("********** End __init__ *************\n")

        # Active-low interrupt → falling edge
        self._int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._on_interrupt)

    def _on_interrupt(self, pin):
        """
        Interrupt handler for active-low H_INTN (int_pin).
        Captures the exact host timestamp (microseconds).
        """
        self.prev_interrupt_us = self.last_interrupt_us
        self.last_interrupt_us = ticks_us()

    def reset_sensor(self):
        if self._reset_pin:
            self.hard_reset()
            reset_type = "Hard"
        else:
            self.soft_reset()
            reset_type = "Soft"

        for attempt in range(3):
            try:
                if self._check_id():
                    self._dbg(f"*** {reset_type} reset successful, acknowledged with 0xF8 response")
                    sleep_ms(100)  # allow SHTP time to settle
                    self._init_complete = True
                    return
            except OSError as e:
                self._dbg(f"Attempt {attempt + 1} failed with OSError: {e}")
            sleep_ms(600)  # TODO BRC revisit if this duration is really needed
        #
        raise RuntimeError(f"Failed to get valid Report ID (0xf8) with {reset_type} reset")

    ############ USER VISIBLE REPORT FUNCTIONS ###########################

    # 3-Tuple Sensor Reports + accuracy + timestamp
    @property
    def linear_acceleration(self):
        """A tuple representing the current linear acceleration values on the X, Y, and Z
        axes in meters per second squared"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_LINEAR_ACCELERATION] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_LINEAR_ACCELERATION]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("linear acceleration report not enabled, use enable_feature") from None

    @property
    def acceleration(self):
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_ACCELEROMETER] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_ACCELEROMETER]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("acceleration report not enabled, use enable_feature") from None

    @property
    def gravity(self):
        """A tuple representing the gravity vector in the X, Y, and Z components
        axes in meters per second squared"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_GRAVITY] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_GRAVITY]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("gravity report not enabled, use enable_feature") from None

    @property
    def gyro(self):
        """A tuple representing Gyro's rotation measurements on the X, Y, and Z
        axes in radians per second"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_GYROSCOPE] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_GYROSCOPE]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("gyroscope report not enabled, use enable_feature") from None

    @property
    def magnetic(self):
        """A tuple of the current magnetic field measurements on the X, Y, and Z axes"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_MAGNETOMETER] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_MAGNETOMETER]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("Magnetometer report not enabled, use enable_feature") from None

    @property
    def raw_acceleration(self):
        """Returns the sensor's raw, unscaled value from the accelerometer registers"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_RAW_ACCELEROMETER] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_RAW_ACCELEROMETER]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("raw acceleration report not enabled, use enable_feature") from None

    @property
    def raw_gyro(self):
        """Returns the sensor's raw, unscaled value from the gyro registers"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_RAW_GYROSCOPE] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_RAW_GYROSCOPE]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("raw gyroscope report not enabled, use enable_feature") from None

    @property
    def raw_magnetic(self):
        """Returns the sensor's raw, unscaled value from the magnetometer registers"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_RAW_MAGNETOMETER] = 0
            x, y, z, acc, ts = self._report_values[BNO_REPORT_RAW_MAGNETOMETER]
            return SensorReading3(x, y, z, acc, ts)
        except KeyError:
            raise RuntimeError("raw magnetic report not enabled, use enable_feature") from None

    @property
    def euler(self):
        # A 3-tuple representing the current Roll, Tilt, and Yaw euler angle in degree
        # q[4] is accuracy, and q[5] is timestamp_us
        self._process_available_packets()
        try:
            self._unread_report_count[self._quaternion_euler_vector] = 0
            q = self._report_values[self._quaternion_euler_vector]
        except KeyError:
            raise RuntimeError("quaternion report not enabled, use enable_feature") from None

        jsqr = q[1] * q[1]
        t0 = +2.0 * (q[3] * q[0] + q[1] * q[2])
        t1 = +1.0 - 2.0 * (q[0] * q[0] + jsqr)
        roll = degrees(atan2(t0, t1))

        t2 = +2.0 * (q[3] * q[1] - q[2] * q[0])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        tilt = degrees(asin(t2))

        t3 = +2.0 * (q[3] * q[2] + q[0] * q[1])
        t4 = +1.0 - 2.0 * (jsqr + q[2] * q[2])
        yaw = degrees(atan2(t3, t4))

        return SensorReading3(roll, tilt, yaw, q[4], q[5])

    # 4-Tuple Sensor Reports + accuracy + timestamp
    @property
    def quaternion(self):
        """A quaternion representing the current rotation vector"""
        self._process_available_packets()
        try:
            # TODO BRC understand
            self._unread_report_count[self._quaternion_euler_vector] = 0
            i, j, k, r, acc, ts = self._report_values[self._quaternion_euler_vector]
            return SensorReading4(i, j, k, r, acc, ts)
        except KeyError:
            raise RuntimeError("quaternion report not enabled, use enable_feature") from None

    @property
    def geomagnetic_quaternion(self):
        """A quaternion representing the current geomagnetic rotation vector"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR] = 0
            i, j, k, r, acc, ts = self._report_values[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]
            return SensorReading4(i, j, k, r, acc, ts)
        except KeyError:
            raise RuntimeError("geomagnetic quaternion report not enabled, use enable_feature") from None

    @property
    def game_quaternion(self):
        """A quaternion representing the current rotation vector expressed as a quaternion with no
        specific reference for heading, while roll and pitch are referenced against gravity. To
        prevent sudden jumps in heading due to corrections, the `game_quaternion` property is not
        corrected using the magnetometer. Some drift is expected"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_GAME_ROTATION_VECTOR] = 0
            i, j, k, r, acc, ts = self._report_values[BNO_REPORT_GAME_ROTATION_VECTOR]
            return SensorReading4(i, j, k, r, acc, ts)
        except KeyError:
            raise RuntimeError("game quaternion report not enabled, use enable_feature") from None

    # Other Sensor Reports
    @property
    def steps(self):
        """The number of steps detected since the sensor was initialized"""
        self._process_available_packets()
        try:
            self._unread_report_count[BNO_REPORT_STEP_COUNTER] = 0
            return self._report_values[BNO_REPORT_STEP_COUNTER]
        except KeyError:
            raise RuntimeError("steps report not enabled, use enable_feature") from None

    @property
    def shake(self):
        """True if a shake was detected on any axis since the last time it was checked
        This property has a "latching" behavior where once a shake is detected, it will stay in a
        "shaken" state until the value is read. This prevents missing shake events but means that
        this property is not guaranteed to reflect the shake state at the moment it is read
        """
        self._process_available_packets()
        try:
            shake_detected = self._report_values[BNO_REPORT_SHAKE_DETECTOR]
            # clear on read
            if shake_detected:
                self._report_values[BNO_REPORT_SHAKE_DETECTOR] = False
            return shake_detected
        except KeyError:
            raise RuntimeError("shake report not enabled, use enable_feature") from None

    @property
    def stability_classification(self):
        """Returns the sensor's assessment of its current stability, one of:
        * "Unknown" - The sensor is unable to classify the current stability
        * "On Table" - The sensor is at rest on a stable surface with very little vibration
        * "Stationary" -  The sensor’s motion is below the stable threshold but\
        the stable duration requirement has not been met. This output is only available when\
        gyro calibration is enabled
        * "Stable" - The sensor’s motion has met the stable threshold and duration requirements.
        * "In motion" - The sensor is moving.
        """
        self._process_available_packets()
        try:
            stability_classification = self._report_values[BNO_REPORT_STABILITY_CLASSIFIER]
            return stability_classification
        except KeyError:
            raise RuntimeError("stability classification report not enabled, use enable_feature") from None

    @property
    def activity_classification(self):
        """Returns the sensor's assessment of the activity that is creating the motions\
        that it is sensing, one of:
        * "Unknown"
        * "In-Vehicle"
        * "On-Bicycle"
        * "On-Foot"
        * "Still"
        * "Tilting"
        * "Walking"
        * "Running"
        * "On Stairs"
        """
        self._process_available_packets()
        try:
            activity_classification = self._report_values[BNO_REPORT_ACTIVITY_CLASSIFIER]
            return activity_classification
        except KeyError:
            raise RuntimeError("activity classification report not enabled, use enable_feature") from None

    def begin_calibration(self) -> None:
        """Begin the sensor's self-calibration routine"""
        # start calibration for accel, gyro, and mag
        self._send_me_command(
            [
                1,  # calibrate accel
                1,  # calibrate gyro
                1,  # calibrate mag
                _ME_CAL_CONFIG,
                0,  # calibrate planar acceleration
                0,  # 'on_table' calibration
                0,  # reserved
                0,  # reserved
                0,  # reserved
            ]
        )
        self._calibration_complete = False

    @property
    def calibration_status(self) -> int:
        """Get the status of the self-calibration"""
        self._send_me_command(
            [
                0,  # calibrate accel
                0,  # calibrate gyro
                0,  # calibrate mag
                _ME_GET_CAL,
                0,  # calibrate planar acceleration
                0,  # 'on_table' calibration
                0,  # reserved
                0,  # reserved
                0,  # reserved
            ]
        )
        return self._magnetometer_accuracy

    def _send_me_command(self, subcommand_params) -> None:
        start_time = ticks_ms()
        local_buffer = self._command_buffer
        _insert_command_request_report(
            _ME_CALIBRATE,
            self._command_buffer,  # should use self._data_buffer :\ but send_packet don't
            self._get_report_seq_id(_COMMAND_REQUEST),
            subcommand_params,
        )
        self._send_packet(_BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(_COMMAND_REQUEST)
        while _elapsed_sec(start_time) < _DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._me_calibration_started_at > start_time:
                break

    def save_calibration_data(self) -> None:
        """Save the self-calibration data"""
        # send a DCD save command
        start_time = ticks_ms()
        local_buffer = bytearray(12)
        _insert_command_request_report(
            _SAVE_DCD,
            local_buffer,  # should use self._data_buffer :\ but send_packet don't
            self._get_report_seq_id(_COMMAND_REQUEST),
        )
        self._send_packet(_BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(_COMMAND_REQUEST)
        while _elapsed_sec(start_time) < _DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")

    # Unimplemented (need to add to processing to reports before ARVR reports can be used
    # @property
    # def arvr_stablized_rotation(self):
    #     """
    #     The ARVR-stabilized rotation vector sensor reports the orientation of the device. Accumulated errprs
    #     are corrected while the device is in motion, which limits discontinuities or jumps in data.
    #     The format of the rotation vector is a unit quaternion plus accuracy estimate (5-tuple).
    #     """
    #     self._process_available_packets()
    #     try:
    #         self._unread_report_count[BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR] = 0
    #         i, j, k, r, ae. acc, ts = self._report_values[BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR]
    #         return SensorReading5(i, j, k, r, ae, acc, ts)
    #     except KeyError:
    #         raise RuntimeError("arvr stabilized rotation report not enabled, use enable_feature") from None
    #
    # @property
    # def arvr_stablized_game_rotation(self):
    #     """
    #     The ARVR-stabilized game rotation vector sensor reports the orientation of the device. Accumulated errprs
    #     are corrected while the device is in motion, which limits discontinuities or jumps in data.
    #     The format of the rotation vector is a unit quaternion.
    #     """
    #     self._process_available_packets()
    #     try:
    #         self._unread_report_count[BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR] = 0
    #         i, j, k, r. acc, ts = self._report_values[BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR]
    #         return SensorReading4(i, j, k, r, ae, acc, ts)
    #     except KeyError:
    #         raise RuntimeError("arvr stabilized game rotation report not enabled, use enable_feature") from None

    ############### private/helper methods ###############

    def _process_available_packets(self, max_packets: int = 10) -> bool:
        """
        Read and handle up to `max_packets` packets while a new interrupt is available.
        safety timeout if data ready stuck
        TODO: > 50 way too long, changed to > 3
        TODO: With this logic, _data_ready stays True until all packets are read.
        If _read_packet() does not return all data for one interrupt,
        we may need a “drain loop” until no more packets exist.
        """
        print(f"begin enable{self._unread_report_count=}")
        processed_count = 0
        start_time = ticks_ms()

        while self._data_ready and processed_count < max_packets:
            try:
                new_packet = self._read_packet(wait=False)
            except PacketError:
                # Transient read errors should not block
                sleep_us(250)
                continue

            if new_packet:
                self._handle_packet(new_packet)
                processed_count += 1
                self._dbg(f"Processed {processed_count} packet{'s' if processed_count > 1 else ''}")

            # Safety timeout to avoid infinite loop if interrupt stuck
            if ticks_diff(ticks_ms(), start_time) > 2:  # ms
                self._dbg("Timeout in _process_available_packets")
                break

        flag = processed_count > 0
        self._dbg(f"_process_available_packets done, {processed_count} packets processed - {flag}")
        print(f"end   enable{self._unread_report_count=}")
        return flag


    def _wait_for_packet_type(self, channel, timeout=3.0):
        """
        Wait for a packet from the specified channel.
        Returns the first valid packet from that channel.
        """
        start_time = ticks_ms()
        while _elapsed_sec(start_time) < timeout:
            try:
                packet = self._read_packet(wait=True)
            except PacketError:
                sleep_ms(10)
                continue

            if packet.header.channel_number == channel:
                return packet

        raise RuntimeError(f"Timeout waiting for packet on channel {channel}")

    def _wait_for_packet(self, channel, report_id=None, timeout=0.5):
        """
        Polls the BNO08x for a specific packet response up to a timeout.

        @param channel: SHTP channel
        @param report_id: specific SHTP ReportID to wait for (optional).
        @param timeout: Timeout duration in seconds.
        @return: received packet.
        @raises: RuntimeError if timeout occurs.
        """
        start_time = ticks_ms()
        while _elapsed_sec(start_time) < timeout:
            # Attempt a non-blocking read via the SPI driver
            # check for INT line and read the SHTP header
            # return the full packet or None/raise an exception if no packet is ready
            try:
                packet = self._read_packet(wait=False)

                if packet is not None:
                    self._handle_packet(packet)

                    if channel == packet.header.channel_number and (report_id is None or report_id == packet.report_id):
                        return packet

            except PacketError:
                pass

            sleep_ms(1)

        raise RuntimeError(
            f"Timed out waiting for packet on channel {channel} with ReportID {report_id} after {timeout}s")

    def _update_sequence_number(self, new_packet: Packet) -> None:
        channel = new_packet.channel_number
        seq = new_packet.header.sequence_number
        self._rx_sequence_number[channel] = seq


    def _handle_packet(self, packet):
        """
        Split a single packet into multiple reports and process them in FIFO order.
        Handles multiple 0xF8 Product ID Response reports correctly.
        """
        data_view = memoryview(packet.data)

        try:
            next_byte_index = 0
            report_count = 0

            while next_byte_index < packet.header.data_length:
                report_id = data_view[next_byte_index]

                # Look up required byte count
                if report_id < 0xF0:
                    required_bytes = _AVAIL_SENSOR_REPORTS[report_id][2]
                else:
                    required_bytes = _REPORT_LENGTHS.get(report_id, 0)
                    if required_bytes == 0:
                        self._dbg(f"Unknown report_id {hex(report_id)}, skipping 1 byte")
                        next_byte_index += 1
                        continue

                unprocessed_byte_count = packet.header.data_length - next_byte_index
                if unprocessed_byte_count < required_bytes:
                    self._dbg(f"Unprocessable batch ERROR: skipping ! {unprocessed_byte_count} bytes")
                    break

                # Zero-copy slice
                report_view = data_view[next_byte_index: next_byte_index + required_bytes]

                # Process immediately (instead of building slices list)
                self._process_report(report_id, report_view)
                report_count += 1

                next_byte_index += required_bytes

            self._dbg(f"HANDLING {report_count} PACKET{'S' if report_count > 1 else ''}...")

        except Exception as error:
            self._dbg(f"Handle Packet: Packet bytes:{[hex(b) for b in packet.data[:4]]}...")
            raise

    def _handle_control_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Handle control reports. Timestamp is first to return quickly
        :param report_id: report ID
        :param report_bytes: portion of packet for report
        :return:
        """
        # Base Timestamp (0xfb)
        if report_id == _BASE_TIMESTAMP:
            self._last_base_timestamp_us = unpack_from("<I", report_bytes, 1)[0] * 100
            self._dbg(f"Base Timestamp (0xfb): {self._last_base_timestamp_us} usec")
            return

        # Timestamp Rebase (0xfa), see this when _BASE_TIMESTAMP wraps so use this instead
        if report_id == _TIMESTAMP_REBASE:
            self._last_base_timestamp_us = unpack_from("<I", report_bytes, 1)[0] * 100
            self._dbg(f"Timestamp Rebase (0xfa): {self._last_base_timestamp_us} usec")
            return

        # Feature response (0xfc)
        if report_id == _GET_FEATURE_RESPONSE:
            _report_id, feature_report_id = unpack_from("<BB", report_bytes)
            self._report_values[feature_report_id] = _INITIAL_REPORTS.get(feature_report_id, (0.0, 0.0, 0.0, 0, 0))
            self._unread_report_count[feature_report_id] = 0
            return

        # Command Response (0xF1)
        if report_id == _COMMAND_RESPONSE:
            self._handle_command_response(report_bytes)
            return

        # Product ID Response (0xf8)
        if report_id == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
            reset_cause = report_bytes[1]
            sw_major = report_bytes[2]
            sw_minor = report_bytes[3]
            sw_part_number = unpack_from("<I", report_bytes, 4)[0]
            sw_build_number = unpack_from("<I", report_bytes, 8)[0]
            sw_patch = unpack_from("<H", report_bytes, 12)[0]

            self._dbg("Product ID Response (0xf8):")
            self._dbg(f"*** Last reset cause: {reset_cause} = {_RESET_CAUSE_STRING[reset_cause]}")
            self._dbg(f"*** Part Number: {sw_part_number}")
            self._dbg(f"*** Software Version: {sw_major}.{sw_minor}.{sw_patch}")
            self._dbg(f"\tBuild: {sw_build_number}\n")

            self._id_read = True
            return

    def _handle_command_response(self, report_bytes: bytearray) -> None:
        (report_body, response_values) = _parse_command_response(report_bytes)
        (_report_id, _seq_number, command, _command_seq_number, _response_seq_number,) = report_body

        # status, accel_en, gyro_en, mag_en, planar_en, table_en, *_reserved) = response_values
        command_status, *_rest = response_values
        self._last_cmd_rsp = (command, command_status)  # TODO BRC is this vestigial? <--- ADD THIS

        if command == _ME_CALIBRATE and command_status == 0:
            self._me_calibration_started_at = ticks_ms()
            self._dbg("_handle_command_response: ME_CALIBRATE success flag set.")

        self.last_cmd_response = {
            "report_id": _COMMAND_RESPONSE,
            "seq": _seq_number,
            "command": command,
            "status": command_status,
        }

        if command == _SAVE_DCD:
            self._dbg(f"_handle_command_response: DCD Save response detected. Status is {command_status}")
            if command_status == _COMMAND_STATUS_SUCCESS:
                self._dcd_saved_at = ticks_ms()
            else:
                raise RuntimeError(f"Unable to save calibration data, status={command_status}")

    def _process_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Process reports both sensor and control reports
        TODO: BRC handle ARVR 5 tuple at end

        Extracted accuracy and delay from sensor report (100usec ticks)
        Multiple reports are processed in the order they appear in the packet buffer.
        Last sensor report's value over-write previous in this packet.
        The first (oldest) report sets self._report_values[report_id],
        """
        # handle typical sensor reports first
        if 0x01 <= report_id <= 0x09:
            sensor_data, accuracy, delay_us = _parse_sensor_report_data(report_bytes)
            self._dbg(f"Report: {reports[report_id]}\nData: {sensor_data}, {accuracy=}, {delay_us=}")

            self._sensor_us = self.last_interrupt_us - self._last_base_timestamp_us + delay_us
            print(
                f"sensor timestamp={self._sensor_us} {(self.last_interrupt_us - self.prev_interrupt_us) / 1000.0} ms")

            if report_id == BNO_REPORT_MAGNETOMETER:
                self._magnetometer_accuracy = accuracy

            self._report_values[report_id] = sensor_data + (accuracy, self._sensor_us)
            self._unread_report_count[report_id] += 1
            return

        # Handle all control reports
        if report_id >= 0xF0:
            self._handle_control_report(report_id, report_bytes)
            return

        if report_id == BNO_REPORT_STEP_COUNTER:
            self._report_values[report_id] = unpack_from("<H", report_bytes, 8)[0]
            return

        if report_id == BNO_REPORT_SHAKE_DETECTOR:
            # shake in X, Y, or Z axes (mask lower 3 bits: 0x07)
            shake_bitfield = unpack_from("<H", report_bytes, 4)[0]
            shake_detected = (shake_bitfield & 0x07) != 0

            # Latch shake in _readings
            if shake_detected:
                previous = self._report_values.get(BNO_REPORT_SHAKE_DETECTOR, False)
                self._report_values[BNO_REPORT_SHAKE_DETECTOR] = True
            return

        if report_id == BNO_REPORT_STABILITY_CLASSIFIER:
            classification_bitfield = unpack_from("<B", report_bytes, 4)[0]
            stability_classification = ["Unknown", "On Table", "Stationary", "Stable", "In motion"][
                classification_bitfield]
            self._report_values[BNO_REPORT_STABILITY_CLASSIFIER] = stability_classification
            return

        # Activitity Classifier in SH-2 (6.5.36)
        if report_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            # 0 Report ID = 0x1E, # 1 Sequence number, # 2 Status, 3 Delay, 4 Page Number + EOS
            # 5 Most likely state, # 6-15 Classification (10 x Page Number) + confidence
            end_and_page_number, most_likely = unpack_from("<BB", report_bytes, 4)
            page_number = end_and_page_number & 0x7F
            confidences = unpack_from("<BBBBBBBBB", report_bytes, 6)
            activity_classification = {"most_likely": ACTIVITIES[most_likely]}
            for idx, raw_confidence in enumerate(confidences):
                confidence = (10 * page_number) + raw_confidence
                activity_string = ACTIVITIES[idx]
                activity_classification[activity_string] = confidence
            self._report_values[BNO_REPORT_ACTIVITY_CLASSIFIER] = activity_classification
            return

        # Raw Magnetometer: returns 4-tuple: x, y, z, and time_stamp
        # Raw accelerometer: returns 4-tuple: x, y, z, and time_stamp
        # time_stamp units in microseconds
        if report_id in (BNO_REPORT_RAW_ACCELEROMETER, BNO_REPORT_RAW_MAGNETOMETER):
            x, y, z = unpack_from("<HHH", report_bytes, 4)
            time_stamp = unpack_from("<I", report_bytes, 12)[0]
            sensor_data = (x, y, z, time_stamp)
            self._report_values[report_id] = sensor_data
            return

        # Raw gyroscope: returns 5-tuple: x, y, z, celsius, and time_stamp
        # time_stamp units in microseconds
        # Celsius float units in celsius
        if report_id == BNO_REPORT_RAW_GYROSCOPE:
            raw_x, raw_y, raw_z, temp_int, time_stamp = unpack_from("<HHHhI", report_bytes, 4)
            celsius = (temp_int / 2.0) + 23.0
            sensor_data = (raw_x, raw_y, raw_z, celsius, time_stamp)
            self._report_values[report_id] = sensor_data
            return

        # General Case all other sensors
        # TODO fix for ARVR 4-tuple and ARVR 5-tuple for ARVR-Stabilized Rotation Vector (0x28)
        sensor_data, accuracy, delay_us = _parse_sensor_report_data(report_bytes)
        self._dbg(f"Report: {reports[report_id]}\nData: {sensor_data}, {accuracy=}, {delay_us=}")
        self._sensor_timestamp = self.last_interrupt_us - self._last_base_timestamp_us + delay_us
        self._report_values[report_id] = sensor_data
        return

    # Enable a given feature of the BNO08x (See Hillcrest 6.5.4)
    def enable_feature(self, feature_id, freq=None):
        """
        Enable sensor features for bno08x, set period in usec (not ms)
        Called recursively since some raw require non-raw to be enabled
        On Channel (0x02), send _SET_FEATURE_COMMAND (0xfb) with feature id await GET_FEATURE_RESPONSE (0xfc)
        """
        self._dbg(f"ENABLING FEATURE ID... {hex(feature_id)}")
        set_feature_report = bytearray(17)
        set_feature_report[0] = _SET_FEATURE_COMMAND
        set_feature_report[1] = feature_id
        
        if freq is None:
            requested_interval = DEFAULT_REPORT_FREQ[feature_id]
        elif freq == 0:
            requested_interval = 0
        else:
            requested_interval = int(1_000_000 / freq)

        pack_into("<I", set_feature_report, 5, requested_interval)

        if feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            pack_into("<I", set_feature_report, 13, _ENABLED_ACTIVITIES)

        feature_dependency = _RAW_REPORTS.get(feature_id, None)
        if feature_dependency and feature_dependency not in self._report_values:
            self._dbg(f" Feature dependency: {feature_dependency}")
            self.enable_feature(feature_dependency, DEFAULT_REPORT_FREQ[feature_dependency])

        # UART operation reqires wake_pin is None
        if self._wake_pin is not None:
            self._dbg("Enable feature WAKE Pulse to ensure BNO08x is out of sleep before INT.")
            self._wake_pin.value(0)
            sleep_ms(2)  # lower than 1ms doesn't seem to work
            self._wake_pin.value(1)
            sleep_ms(10)  # 1 ms works, 1 ms sometimes fails

        self._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)

        try:
            # TODO BRC May need to revert to general Packet handling since previous enabled functions
            # already start sending reports and this may get mixed up
            report_bytes = self._wait_for_packet(_BNO_CHANNEL_CONTROL, _GET_FEATURE_RESPONSE,
                                                 timeout=_FEATURE_ENABLE_TIMEOUT)
            data = report_bytes.data
            fid = data[1]
            report_interval = unpack_from("<I", data, 5)[0]
            
            self._report_values[feature_id] = _INITIAL_REPORTS.get(feature_id, (0.0, 0.0, 0.0, 0, 0))
            self._report_periods_dictionary_us[feature_id] = report_interval
            self._dbg(f"Report enabled: {_REPORTS_DICTIONARY[fid]}: {hex(fid)}")
            self._dbg(f" Initial tuple={self._report_values}")
            self._dbg(f" Requested Interval: {requested_interval / 1000.0:.1f} ms")
            self._dbg(f" Actual    Interval: {report_interval / 1000.0:.1f} ms")
            return

        except RuntimeError:
            raise RuntimeError(f"BNO08X: enable_feature: not able to enable feature: {feature_id}")

    def report_period_us(self, feature_id):
        """
        return
        """
        return self._report_periods_dictionary_us[feature_id]

    def print_report_period(self):
        print(f"Enabled Report Periods:")
        for feature_id in self._report_periods_dictionary_us.keys():
            period_ms = self.report_period_us(feature_id) / 1000.0
            print(f"\t{feature_id}: {_REPORTS_DICTIONARY[feature_id]},\t{period_ms:.1f} ms, {1_000 / period_ms:.1f} Hz")

    def set_orientation(self, quaternion):
        return  # Procedure to be completed and corrected
        # set orientation of the system
        self._dbg("DEVICE ORIENTATION SETTING UP...")
        set_orientation = bytearray(17)
        set_orientation[0] = FRS_WRITE_REQUEST
        set_orientation[1] = 0,  # reserved
        set_orientation[2] = 0,  # Length LSB
        set_orientation[3] = BNO_CONF_SYSTEM_ORIENTATION & 0xFF,  # FRS Type LSB
        set_orientation[4] = BNO_CONF_SYSTEM_ORIENTATION >> 80,  # FRS Type MSB

        self._send_packet(BNO_CHANNEL_CONTROL, set_orientation)

        set_orientation[0] = FRS_WRITE_DATA
        set_orientation[1] = 0,  # reserved
        set_orientation[2] = 0,  # Offset LSB
        set_orientation[3] = 0,  # Offset MSB
        set_orientation[4] = ORENT_QW & 0xFF,  # Data0 LSB
        set_orientation[5] = ORENT_QW >> 8,  # Data0 MSB
        set_orientation[6] = 0,  # Offset LSB
        set_orientation[7] = 0,  # Offset MSB
        set_orientation[8] = ORENT_QX & 0xFF,  # Data1 LSB
        set_orientation[9] = ORENT_QX >> 8,  # Data1 MSB
        set_orientation[10] = 0,  # Offset LSB
        set_orientation[11] = 0,  # Offset MSB
        set_orientation[12] = ORENT_QY & 0xFF,  # Data2 LSB
        set_orientation[13] = ORENT_QY >> 8,  # Data2 MSB
        set_orientation[14] = 0,  # Offset LSB
        set_orientation[15] = 0,  # Offset MSB
        set_orientation[16] = ORENT_QZ & 0xFF,  # Data2 LSB
        set_orientation[17] = ORENT_QZ >> 8,  # Data2 MSB

        self._send_packet(BNO_CHANNEL_CONTROL, set_orientation)

    def set_quaternion_euler_vector(self, feature_id):
        self._quaternion_euler_vector = feature_id
        return

    def _check_id(self):
        """
        Ensure the BNO08X product ID is read.
        Handles multiple 0xF8 responses in a single packet.
        """
        self._dbg("********** Check ID **********")
        if getattr(self, "_id_read", False):
            return True

        # Send Product ID request
        data = bytearray(2)
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST
        data[1] = 0
        self._send_packet(_BNO_CHANNEL_CONTROL, data)

        start_time = ticks_ms()
        while _elapsed_sec(start_time) < 3.0:
            try:
                packet = self._wait_for_packet_type(_BNO_CHANNEL_CONTROL)
                # Handle all reports in the packet
                self._handle_packet(packet)

                # Check if any 0xF8 was processed
                if getattr(self, "_id_read", False):
                    return True
            except RuntimeError:
                pass
            except PacketError:
                pass

        raise RuntimeError("_check_id: Timeout waiting for valid Product ID response")

    def _dbg(self, *args, **kwargs) -> None:
        if self._debug:
            print("DBG::\t\t", *args, **kwargs)


    def hard_reset(self) -> None:
        """Hardware reset the sensor to an initial unconfigured state"""
        if not self._reset_pin:
            return

        self._dbg("*** Hard Reset start (base class: spi, i2c)...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_us(10)  # sleep_us(1), data sheet say only 10ns required,
        self._reset_pin.value(1)
        sleep_ms(200)  # orig was 10ms, datasheet implies 94 ms required
        self._dbg("*** Hard Reset End, awaiting Acknowledgement")

    def soft_reset(self) -> None:
        """Reset the sensor to an initial unconfigured state"""
        self._dbg("Start SOFT RESET...")

        data = bytearray(1)
        data[0] = 1
        _seq = self._send_packet(BNO_CHANNEL_EXE, data)
        sleep_ms(500)
        _seq = self._send_packet(BNO_CHANNEL_EXE, data)
        sleep_ms(500)

        for _i in range(3):
            try:
                _packet = self._read_packet()
            except PacketError:
                sleep_ms(500)

        self._dbg("End Soft RESET, awaiting Acknowledgement")

    def _send_packet(self, channel, data):
        raise RuntimeError("_send_packet Not implemented in bno08x.py, supplanted by I2C or SPI subclass")

    def _read_packet(self):
        raise RuntimeError("_read_packet Not implemented in bno08x.py, supplanted by I2C or SPI subclass")

    # TODO change these to using self._tx_sequence_number, likely better to put into methods that use them
    def _increment_report_seq(self, report_id: int) -> None:
        current = self._two_ended_sequence_numbers.get(report_id, 0)
        self._two_ended_sequence_numbers[report_id] = (current + 1) % 256

    def _get_report_seq_id(self, report_id: int) -> int:
        return self._two_ended_sequence_numbers.get(report_id, 0)

    # TODO add to I2C and UART
    @property
    def _data_ready(self):
        """
        Returns True if at least one new interrupt seen based on timestamps
        """
        if self.last_interrupt_us != self.prev_interrupt_us:
            return True
        return False
