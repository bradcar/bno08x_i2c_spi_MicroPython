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

Driver library for  BNO08x IMUs by CEVA  Hillcrest Laboratories
* Author(s): bradcar - 100% inspired by Bryan & dobodu
* Author(s): Bryan Siepert
* Author(s): dobodu

Implementation Notes
--------------------

**Hardware:**

* bno086

**Software and Dependencies:**

* MicroPython

Using interrupt pin on all sensor types so can accuractly get timestamp of sensor readings

Timing of sensor reports:
* first interrupt defines sensor epoch, all sensor report timestamps (ms, microseonds)
  are relative to this first interrupt.
* self._sensor_epoch_ms starts at 0.0 ms at first interrupt.
* self._epoch_start_ms was the host ticks ms at first interrupt.

Delay
1. When the timebase reference report is provided with individual sensor report
   it will likely have a delay of zero.
2. In cases where sensor reports are concatenated (due to delays in processing),
   the delay field may be populated, then delay and the timebase reference
   are used to calculate the sensor sample's actual timestamp.

Current best sensor update periods - BNO086 responded with 2ms update frequeny:
- spi:   2.1ms (476 Hz)
- i2c:   3.3ms (303 Hz)
- uart:  ?.?ms ( ?? Hz)

TODO: decide on call functions, wait for new data, or return what have
TODO: How to handle unimplemented reports that are sent by sensor? Pass them without error?
TODO: apply spi optimizations to uart ?  fix UART mis-framing (with quaternions?)
TODO: test UART with Reset & Interrupt pins

Possible future projects:
FUTURE: explore adding simple 180 degree calibration(0x0c), page 55 SH-2, but will need move request reports
FUTURE: include estimated ange in full quaternion implementation, maybe make new modifier bno.quaternion.est_angle
FUTURE: process two ARVR reports (rotation vector has estimated angle which has a different Q-point)
"""

__version__ = "0.9.1"
__repo__ = "https://github.com/bradcar/bno08x_i2c_spi_MicroPython"

from math import asin, atan2, degrees
from struct import pack_into, unpack_from, pack

import uctypes
from collections import namedtuple
from machine import Pin
from micropython import const
from utime import ticks_ms, ticks_us, ticks_diff, sleep_ms, sleep_us

# Commands
SHTP_CHAN_COMMAND = const(0)
SHTP_CHAN_EXE = const(1)
SHTP_CHAN_CONTROL = const(2)
SHTP_CHAN_INPUT = const(3)
SHTP_CHAN_WAKE_INPUT = const(4)
BNO_CHAN_GYRO_ROTATION_VECTOR = const(5)

channels = {
    0x0: "SHTP_CHAN_COMMAND",
    0x1: "SHTP_CHAN_EXE",
    0x2: "SHTP_CHAN_CONTROL",
    0x3: "SHTP_CHAN_INPUT",
    0x4: "SHTP_CHAN_WAKE_INPUT",
    0x5: "BNO_CHAN_GYRO_ROTATION_VECTOR",
}

_SHTP_HEADER_LEN = 4

_GET_FEATURE_REQUEST = const(0xFE)
_SET_FEATURE_COMMAND = const(0xFD)
_GET_FEATURE_RESPONSE = const(0xFC)
_BASE_TIMESTAMP = const(0xFB)
_TIMESTAMP_REBASE = const(0xFA)
_REPORT_PRODUCT_ID_REQUEST = const(0xF9)
_REPORT_PRODUCT_ID_RESPONSE = const(0xF8)
_FRS_WRITE_REQUEST = const(0xF7)
_FRS_WRITE_DATA = const(0xF6)
_FRS_WRITE_RESPONSE = const(0xF5)
_FRS_READ_REQUEST = const(0xF4)
_FRS_READ_RESPONSE = const(0xF3)
_COMMAND_REQUEST = const(0xF2)
_COMMAND_RESPONSE = const(0xF1)

# Sensor Commands
_COMMAND_ADVERTISE = const(0x00)  # Request Advertisement command on Chan 0
_COMMAND_RESET = const(0x01)  # Soft Reset command on Chan 0

# Status Constants
_COMMAND_STATUS_SUCCESS = 0

# ME / DCD Calibration commands and sub-commands
_ME_TARE_COMMAND = const(0x03)
_SAVE_DCD_COMMAND = const(0x06)
_ME_CALIBRATE_COMMAND = const(0x07)

# ME/DCD Subcommads
_ME_CAL_CONFIG = const(0x00)
_ME_GET_CAL = const(0x01)
_ME_TARE_NOW = const(0x00)
_ME_PERSIST_TARE = const(0x01)
_ME_TARE_SET_REORIENTATION = const(0x02)

# Reports Summary depending on BNO device
BNO_REPORT_ACCELEROMETER = const(0x01)  # bno.acceleration (m/s^2, gravity acceleration included)
BNO_REPORT_GYROSCOPE = const(0x02)  # bno.gyro (rad/s).
BNO_REPORT_MAGNETOMETER = const(0x03)  # bno.magnetic (in µTesla).
BNO_REPORT_LINEAR_ACCELERATION = const(0x04)  # bno.linear_acceleration (m/^2, no gravity acceleration)
BNO_REPORT_ROTATION_VECTOR = const(0x05)  # bno.quaternion
BNO_REPORT_GRAVITY = const(0x06)  # bno.gravity
BNO_REPORT_UNCALIBRATED_GYROSCOPE = const(0x07)
BNO_REPORT_GAME_ROTATION_VECTOR = const(0x08)  # bno.game_quaternion
BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = const(0x09)  # bno.geomagnetic_quaternion
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
BNO_REPORT_MOTION_REQUEST = const(0x2B)
BNO_REPORT_OPTICAL_FLOW = const(0x2c)
BNO_REPORT_DEAD_RECKONING = const(0x2d)

_REPORTS_DICTIONARY = {
    0x01: "acceleration",
    0x02: "gyro",
    0x03: "magnetic",
    0x04: "linear_acceleration",
    0x05: "quaternion",
    0x06: "gravity",
    0x07: "UNCALIBRATED_GYROSCOPE",
    0x08: "game_quaternion",
    0x09: "geomagnetic_quaternion",
    0x0A: "PRESSURE",
    0x0B: "AMBIENT LIGHT",
    0x0C: "HUMIDITY",
    0x0D: "PROXIMITY",
    0x0E: "TEMPERATURE",
    0x0F: "UNCALIBRATED_MAGNETIC_FIELD",
    0x10: "TAP_DETECTOR",
    0x11: "steps",
    0x12: "SIGNIFICANT_MOTION",
    0x13: "stability_classifier",
    0x14: "raw_acceleration",
    0x15: "raw_gyro",
    0x16: "raw_magnetic",
    0x17: "SAR - reserved",
    0x18: "steps",
    0x19: "SHAKE_DETECTOR",
    0x1A: "FLIP_DETECTOR",
    0x1B: "PICKUP_DETECTOR",
    0x1C: "STABILITY_DETECTOR",
    0x1D: "0X1D unknown",
    0x1E: "activity_classifier",
    0x1F: "SLEEP_DETECTOR",
    0x20: "TILT_DETECTOR",
    0x21: "POCKET_DETECTOR",
    0x22: "CIRCLE_DETECTOR",
    0x23: "HEART_RATE_MONITOR",
    0x28: "ARVR_STABILIZED_ROTATION_VECTOR",
    0x29: "ARVR_STABILIZED_GAME_ROTATION_VECTOR",
    0x2A: "GYRO INTEGRATED_ROTATION_VECTOR",
    0x2B: "MOTION_REQUEST",
    0x2C: "OPTICAL_FLOW",
    0x2D: "DEAD_RECKONING",
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

_MAX_PACKET_PROCESS = 10

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
    # BNO_REPORT_PRESSURE: 2,
    # BNO_REPORT_AMBIENT_LIGHT: 10,
    # BNO_REPORT_HUMIDITY: 2,
    # BNO_REPORT_PROXIMITY: 10,
    # BNO_REPORT_TEMPERATURE: 2,
    BNO_REPORT_STEP_COUNTER: 5,
    # BNO_REPORT_SHAKE_DETECTOR: 20,
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

# pre-calculate the reciprocals
_Q_POINT_14_SCALAR = 2 ** (14 * -1)
_Q_POINT_12_SCALAR = 2 ** (12 * -1)
_Q_POINT_9_SCALAR = 2 ** (9 * -1)
_Q_POINT_8_SCALAR = 2 ** (8 * -1)
_Q_POINT_4_SCALAR = 2 ** (4 * -1)

_REPORT_LENGTHS = {
    # Sensor Reports
    BNO_REPORT_ACCELEROMETER: 10,  # 0x01
    BNO_REPORT_GYROSCOPE: 10,  # 0x02
    BNO_REPORT_MAGNETOMETER: 10,  # 0x03
    BNO_REPORT_LINEAR_ACCELERATION: 10,  # 0x04
    BNO_REPORT_ROTATION_VECTOR: 14,  # 0x05
    BNO_REPORT_GRAVITY: 10,  # 0x06
    BNO_REPORT_UNCALIBRATED_GYROSCOPE: 16,  # For testing #07
    BNO_REPORT_GAME_ROTATION_VECTOR: 12,  # 0x08
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: 14,  # 0x09
    #     BNO_REPORT_PRESSURE: 8,  #0x0a
    #     BNO_REPORT_AMBIENT_LIGHT: 8,  #0x0b
    #     BNO_REPORT_HUMIDITY: 6, #0x0c
    #     BNO_REPORT_PROXIMITY: 6, #0x0d
    #     BNO_REPORT_TEMPERATURE: 6, #0x0e
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER: 16,  # For testing,  # 0x0f
    #     BNO_REPORT_TAP_DETECTOR: 5, # 0x10
    BNO_REPORT_STEP_COUNTER: 12,  # 0x11
    #     BNO_REPORT_SIGNIFICANT_MOTION: 6, # 0x12
    BNO_REPORT_STABILITY_CLASSIFIER: 6,  # 0x13
    BNO_REPORT_RAW_ACCELEROMETER: 16,  # 0x14
    BNO_REPORT_RAW_GYROSCOPE: 16,  # 0x15
    BNO_REPORT_RAW_MAGNETOMETER: 16,  # 0x16
    #     BNO_REPORT_SAR reserved  # 0x17
    BNO_REPORT_STEP_DETECTOR: 8,  # 0x18
    #     BNO_REPORT_SHAKE_DETECTOR: 6,  # 0x19
    #     BNO_REPORT_FLIP_DETECTOR: 6,  # 0x1a
    #     BNO_REPORT_PICKUP_DETECTOR: 6,  # 0x1b
    BNO_REPORT_STABILITY_DETECTOR: 6,  # 0x1c
    BNO_REPORT_ACTIVITY_CLASSIFIER: 16,  # 0x1e
    #     BNO_REPORT_SLEEP_DETECTOR: 6,   # 0x1f
    #     BNO_REPORT_TILT_DETECTOR: 6,   # 0x20
    #     BNO_REPORT_POCKET_DETECTOR: 6,  # 0x21
    #     BNO_REPORT_CIRCLE_DETECTOR: 6,  #0x22
    #     BNO_REPORT_HEART_RATE_MONITOR: 6,  #0x23
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: 14,  # 0x28
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: 12,  # 0x29
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: 14,  # 0x2a
    #     BNO_REPORT_MOTION_REQUEST: 6,  # sent to host periodically? 0x2b
    #     BNO_REPORT_OPTICAL_FLOW: 24,  #  0x2c
    #     BNO_REPORT_DEAD_RECKONING: 60, #  0x2d
    # Command Reports
    _COMMAND_RESPONSE: 16,  # 0xf1
    _REPORT_PRODUCT_ID_RESPONSE: 16, # 0xf8
    _GET_FEATURE_RESPONSE: 17,  # 0xfc
    _BASE_TIMESTAMP: 5,  # 0xfb
    _TIMESTAMP_REBASE: 5,  #0xfa
}

# these raw reports require their counterpart to be enabled
_RAW_REPORTS = {
    BNO_REPORT_RAW_ACCELEROMETER: BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE: BNO_REPORT_GYROSCOPE,
    BNO_REPORT_UNCALIBRATED_GYROSCOPE: BNO_REPORT_GYROSCOPE,  # For testing
    BNO_REPORT_RAW_MAGNETOMETER: BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER: BNO_REPORT_MAGNETOMETER,  # For testing
}

# cause of the processor reset
_RESET_CAUSE_STRING = [
    "Not Applicable",
    "Power On Reset",
    "Internal System Reset",  # soft reset by sending command
    "Watchdog Timeout",
    "External Reset",  # hard reset, int_pin
    "Other",
]

# sensor reports (scalar, #results (without .full), bytes in sensor report packet
_SENSOR_SCALING = {
    BNO_REPORT_ACCELEROMETER: (_Q_POINT_8_SCALAR, 3),  # 0x01
    BNO_REPORT_GYROSCOPE: (_Q_POINT_9_SCALAR, 3),  # 0x02
    BNO_REPORT_MAGNETOMETER: (_Q_POINT_4_SCALAR, 3),  # 0x03
    BNO_REPORT_LINEAR_ACCELERATION: (_Q_POINT_8_SCALAR, 3),  # 0x04
    BNO_REPORT_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4),  # 0x05
    BNO_REPORT_GRAVITY: (_Q_POINT_8_SCALAR, 3),  # 0x06
    BNO_REPORT_UNCALIBRATED_GYROSCOPE: (_Q_POINT_9_SCALAR, 3),  # For testing #07
    BNO_REPORT_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4),  # 0x08
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (_Q_POINT_12_SCALAR, 4),  # 0x09
    #     BNO_REPORT_PRESSURE: (1, 1),  #0x0a
    #     BNO_REPORT_AMBIENT_LIGHT: (1, 1),  #0x0b
    #     BNO_REPORT_HUMIDITY: (1, 1), #0x0c
    #     BNO_REPORT_PROXIMITY: (1, 1), #0x0d
    #     BNO_REPORT_TEMPERATURE: (1, 1), #0x0e
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER: (_Q_POINT_4_SCALAR, 3),  # For testing,  # 0x0f
    #     BNO_REPORT_TAP_DETECTOR: (1, 1), # 0x10
    BNO_REPORT_STEP_COUNTER: (1, 1),  # 0x11
    #     BNO_REPORT_SIGNIFICANT_MOTION: (1, 1), # 0x12
    BNO_REPORT_STABILITY_CLASSIFIER: (1, 1),  # 0x13
    BNO_REPORT_RAW_ACCELEROMETER: (1, 3),  # 0x14
    BNO_REPORT_RAW_GYROSCOPE: (1, 3),  # 0x15
    BNO_REPORT_RAW_MAGNETOMETER: (1, 3),  # 0x16
    #     BNO_REPORT_SAR reserved  # 0x17
    BNO_REPORT_STEP_DETECTOR: (1, 1),  # 0x18
    #     BNO_REPORT_SHAKE_DETECTOR: (1, 1),  # 0x19
    #     BNO_REPORT_FLIP_DETECTOR: (1, 1),  # 0x1a
    #     BNO_REPORT_PICKUP_DETECTOR: (1, 1),  # 0x1b
    BNO_REPORT_STABILITY_DETECTOR: (1, 1),  # 0x1c
    # 0x1d ???
    BNO_REPORT_ACTIVITY_CLASSIFIER: (1, 1),  # 0x1e
    #     BNO_REPORT_SLEEP_DETECTOR: (1, 1),   # 0x1f
    #     BNO_REPORT_TILT_DETECTOR: (1, 1),   # 0x20
    #     BNO_REPORT_POCKET_DETECTOR: (1, 1),  # 0x21)
    #     BNO_REPORT_CIRCLE_DETECTOR: (1, 1),  #0x22)
    #     BNO_REPORT_HEART_RATE_MONITOR: (1, 1),  #0x23
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 5),  # 0x28, note est acc QPoint 12 ?
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4),  # 0x29
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4),  # #2a
    #     BNO_REPORT_MOTION_REQUEST: (1, 1),  # sent to host periodically? 0x2b
    #     BNO_REPORT_OPTICAL_FLOW: (1 ,1),  #  0x2c
    #     BNO_REPORT_DEAD_RECKONING: (1 ,1), #  0x2d
}

# uctypes layout for standard BNO08x sensor reports
_SENSOR_REPORT_LAYOUT = {
    "report_id": 0 | uctypes.UINT8,
    "status": 1 | uctypes.UINT8,

    # byte2 contains accuracy+delay high bits, byte 3 has delay low bits
    "byte2": 2 | uctypes.UINT8,
    "byte3": 3 | uctypes.UINT8,

    "v1": 4 | uctypes.INT16,
    "v2": 6 | uctypes.INT16,
    "v3": 8 | uctypes.INT16,  # valid for 3-tuple reports
    "v4": 10 | uctypes.INT16,  # valid for 4-tuple reports (quaternion)
    "e1": 12 | uctypes.INT16,  # valid for rotation & ARVR rotation: quaternion + angle estimate
}

_INITIAL_REPORTS = {
    BNO_REPORT_ACTIVITY_CLASSIFIER: ("Unknown", 0),
    BNO_REPORT_STABILITY_CLASSIFIER: "Unknown",
    BNO_REPORT_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0.0),
    BNO_REPORT_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0.0),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0.0),
    # Gyro is a 5 tuple, Celsius float and int timestamp for last two entry
    BNO_REPORT_RAW_GYROSCOPE: (0, 0, 0, 0.0, 0),
    # Acc & Mag are 4-tuple, int timestamp for last entry
    BNO_REPORT_RAW_ACCELEROMETER: (0, 0, 0, 0),
    BNO_REPORT_RAW_MAGNETOMETER: (0, 0, 0, 0),
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0),
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0, 0.0),
    BNO_REPORT_STEP_COUNTER: 0,
}

# Activity classifier Initialization
ACTIVITIES = ["Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still", "Tilting", "Walking", "Running", "OnStairs", ]
_ENABLED_ACTIVITIES = 0x1FF  # Enable 9  activities: 1 bit set for each of 8 activities and 1 Unknown

DATA_BUFFER_SIZE = const(512)  # data buffer size. obviously eats ram
PacketHeader = namedtuple(
    "PacketHeader",
    ["packet_byte_count", "channel_number", "sequence_number", "report_id_number", ],
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
def _elapsed_sec(ticks_start):
    """ Elapsed time between now - ticks_start. Returns float in seconds """
    return ticks_diff(ticks_ms(), ticks_start) / 1000.0


############ COMMAND PARSING ###########################
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
    """ A class representing a Sensor Hub Transport Packet (4-byte headers) """

    def __init__(self, packet_sh2: bytearray) -> None:
        """header = PacketHeader(packet_byte_count, channel_number, sequence_number, report_id_number)"""
        self.header = self.header_from_buffer(packet_sh2)
        # self.data = packet_sh2[4:self.byte_count]
        self.packet_sh2 = packet_sh2

    def __str__(self) -> str:
        length = self.byte_count
        outstr = "\n\t\t********** Packet *************\n"
        outstr += "DBG::\t\tHeader:\n"
        outstr += f"DBG::\t\t Packet Len: {length} ({hex(length)})\n"
        outstr += f"DBG::\t\t Channel: {channels[self.channel]} ({hex(self.channel)})\n"
        outstr += f"DBG::\t\t Sequence: {self.seq}\n"

        if self.channel in {SHTP_CHAN_CONTROL, SHTP_CHAN_INPUT, }:
            if self.report_id in _REPORTS_DICTIONARY:
                outstr += f"DBG::\t\t Report Type: {_REPORTS_DICTIONARY[self.report_id]} ({hex(self.report_id)})\n"
            else:
                outstr += f"DBG::\t\t \t** UNKNOWN Report Type **: {hex(self.report_id)}\n"
            if self.report_id == 0xFC and length - _SHTP_HEADER_LEN >= 6 and self.report_id in _REPORTS_DICTIONARY:
                # first report_id (self.data[0]), the report type to be enabled (self.data[1])
                outstr += f"DBG::\t\t Feature Enabled: {_REPORTS_DICTIONARY[self.packet_sh2[5]]} ({hex(self.packet_sh2[5])})\n"

        outstr += "\nDBG::\t\tData:\n"
        outstr += f"DBG::\t\t Data Len: {length - _SHTP_HEADER_LEN}"
        for idx, packet_byte in enumerate(self.packet_sh2[4:length]):
            packet_index = idx + _SHTP_HEADER_LEN
            if (packet_index % 4) == 0:
                outstr += f"\nDBG::\t\t[0x{packet_index:02X}] "
            outstr += f"0x{packet_byte:02X} "
        outstr += "\n\t\t*******************************\n"
        # ascii = ''.join(chr(b) if 32 <= b <= 126 else f" x{b:02X}" for b in self.packet_sh2[:length])
        # outstr += f"\nDBG::\t\t ascii: {ascii}\n"

        # preliminary decoding of packets
        if self.byte_count - _SHTP_HEADER_LEN == 15 and self.channel == SHTP_CHAN_INPUT and self.report_id == 0xfb:
            outstr += f"DBG::\t\t first report: {_REPORTS_DICTIONARY[self.packet_sh2[9]]} ({hex(self.packet_sh2[9])})\n"

        if self.byte_count - _SHTP_HEADER_LEN == 1 and self.channel == SHTP_CHAN_EXE and self.report_id == 0x01:
            outstr += "DBG::\t\t Command Execution Response: SHTP_COMMAND (0x0)"
            outstr += "\nDBG::\t\t - Reset Complete Acknowledged, 0xf8 reports to follow\n"

        # Advertisement Response provides sensor information that is printed with debug=True
        # I2C & UART use New Style Advertisement
        if self.byte_count - _SHTP_HEADER_LEN == 51 and self.channel == SHTP_CHAN_COMMAND and self.report_id == _COMMAND_ADVERTISE:
            outstr += "DBG::\t\tNew Style SHTP Advertisement Response (0x00), channel: SHTP_COMMAND (0x0)\n"
            length = len(self.packet_sh2)
            index = _SHTP_HEADER_LEN + 1

            # Tag Processors: {tag_id: (name, format, subtract_header_4, clamp_max_1024)}
            tag_dictionary = {0: ("TAG_NULL", 'S', 0, 0), 1: ("TAG_GUID", '<I', 0, 0),
                              2: ("Max Cargo Write", '<H', 1, 0),
                              3: ("Max Cargo Read", '<H', 1, 0), 4: ("TAG_MAX_TRANSFER_WRITE", '<H', 0, 1),
                              5: ("TAG_MAX_TRANSFER_READ", '<H', 0, 1), 6: ("TAG_NORMAL_CHANNEL", '<B', 0, 0),
                              7: ("TAG_WAKE_CHANNEL", '<B', 0, 0), 8: ("TAG_APP_NAME", 'S', 0, 0),
                              9: ("TAG_CHANNEL_NAME", 'S', 0, 0),
                              10: ("TAG_ADV_COUNT", '<B', 0, 0), 0x80: ("Version", 'S', 0, 0)}

            while index < length:
                tag, tag_len = self.packet_sh2[index:index + 2]
                value_index = index + 2
                next_index = value_index + tag_len
                value = self.packet_sh2[value_index:next_index]
                index = next_index

                if tag not in tag_dictionary:
                    outstr += f"Uknown tag = {tag}\n"
                    continue

                name, fmt, sub_hdr, clamp = tag_dictionary[tag]
                if fmt == 'S':
                    s = "" if tag == 0 else f": {value.decode('ascii')}"
                    outstr += f"DBG::\t\t {name}{s}\n"
                else:
                    v = unpack_from(fmt, value)[0]
                    if sub_hdr:
                        v -= 4
                        s = ", without header"
                    else:
                        s = ""
                    if clamp: v = min(v, 1024)
                    outstr += f"DBG::\t\t {name}: {v}{s}\n"

            return outstr

        # Advertisement Response provides sensor information that is printed with debug=True
        # SPI uses New Style Advertisement
        if self.byte_count - _SHTP_HEADER_LEN == 34 and self.channel == SHTP_CHAN_COMMAND and self.report_id == _COMMAND_ADVERTISE:
            outstr += "DBG::\t\tOld Style SHTP Advertisement Response (0x00), channel: SHTP_COMMAND (0x0)\n"
            p = _SHTP_HEADER_LEN + 4
            response_id = self.packet_sh2[p]
            normal_channel = self.packet_sh2[p + 1]
            wake_channel = self.packet_sh2[p + 2]
            max_cargo_write = self.packet_sh2[p + 3] | (self.packet_sh2[p + 4] << 8)
            max_cargo_read = self.packet_sh2[p + 5] | (self.packet_sh2[p + 6] << 8)
            max_transfer_write = self.packet_sh2[p + 7] | (self.packet_sh2[p + 8] << 8)
            max_transfer_read = self.packet_sh2[p + 9] | (self.packet_sh2[p + 10] << 8)
            outstr += f"DBG::\t\t Normal Channel: {normal_channel}\n"
            outstr += f"DBG::\t\t Wake Channel: {wake_channel}\n"
            outstr += f"DBG::\t\t Max Cargo Write: {max_cargo_write}\n"
            outstr += f"DBG::\t\t Max Cargo Read:  {max_cargo_read}\n"
            outstr += f"DBG::\t\t Max Transfer Write: {max_transfer_write}\n"
            outstr += f"DBG::\t\t Max Transfer Read: {max_transfer_read}\n"
            idx = p + 11  # strings, null terminated
            end = self.byte_count

            def read_cstring(buf, start):
                i = start
                while i < end and buf[i] != 0:
                    i += 1
                s = buf[start:i].decode("ascii", "ignore")
                return s, i + 1

            app_name, idx = read_cstring(self.packet_sh2, idx)
            chan_name, idx = read_cstring(self.packet_sh2, idx)
            ctl_name, idx = read_cstring(self.packet_sh2, idx)
            outstr += f"DBG::\t\t App: {app_name}\n"
            outstr += f"DBG::\t\t Channel: {chan_name}\n"
            outstr += f"DBG::\t\t Controller: {ctl_name}\n"
        return outstr

    @property
    def byte_count(self) -> int:
        """The packet channel"""
        return self.header.packet_byte_count

    @property
    def channel(self) -> int:
        """The packet channel"""
        return self.header.channel_number

    @property
    def seq(self) -> int:
        """The packet seq number"""
        return self.header.sequence_number

    @property
    def report_id(self) -> int:
        """The Packet's first Report ID"""
        return self.header.report_id_number

    @classmethod
    def header_from_buffer(cls, packet_bytes: bytearray) -> PacketHeader:
        """
        Creates a `PacketHeader` object from a given buffer, BNO datasheet 5.2.2
        First 4 bytes are actual SHTP header, 5th byte is the first report_id
        """
        header_data = unpack_from("<HBBB", packet_bytes, 0)
        packet_byte_count = header_data[0] & ~0x8000
        channel_number = header_data[1]
        sequence_number = header_data[2]
        report_id_number = header_data[3]
        header = PacketHeader(packet_byte_count, channel_number, sequence_number, report_id_number)
        return header

    @classmethod
    def is_error(cls, header: PacketHeader) -> bool:
        """Returns True if the header is an error condition"""
        if header.channel_number > 5:
            return True
        if header.packet_byte_count == 0xFFFF and header.sequence_number == 0xFF:
            return True
        return False


class SensorFeature1:
    """ 1-tuple feature manager with methods for enable and reading"""
    __slots__ = ("_bno", "feature_id")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id

    def enable(self, hertz=None):
        """Method to enable the feature with a given report rate (hertz)."""
        return self._bno.enable_feature(self.feature_id, hertz)

    @property
    def value(self):
        try:
            return self._bno._report_values[self.feature_id]
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    def __iter__(self):
        yield self.value

    def __repr__(self):
        return f"{self.value}"


class SensorFeature2:
    """ 2-tuple feature manager with methods for enable, reading, and metadata."""
    __slots__ = ("_bno", "feature_id")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id

    def enable(self, hertz=None):
        """Method to enable the feature with a given report rate (hertz)."""
        return self._bno.enable_feature(self.feature_id, hertz)

    # This method allows the object to be implicitly converted to the (v1, v2) tuple.
    def __iter__(self):
        """Allows direct unpacking: x, y = bno.activity_classifier"""
        try:
            x, y = self._bno._report_values[self.feature_id]
            return iter((x, y))
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None


class SensorFeature3:
    """ 3-tuple feature manager with methods for enable, reading, and metadata."""
    __slots__ = ("_bno", "feature_id")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id

    def enable(self, hertz=None):
        """Method to enable the feature with a given report rate (hertz)."""
        return self._bno.enable_feature(self.feature_id, hertz)

    # --- Properties to retrieve the latest data from the BNO instance ---
    @property
    def meta(self):
        """Returns (accuracy, timestamp_ms)."""
        try:
            _, _, _, acc, ts = self._bno._report_values[self.feature_id]
            return acc, ts
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    @property
    def full(self):
        """Returns (v1, v2, v3, accuracy, timestamp_ms)."""
        try:
            self._bno._unread_report_count[self.feature_id] = 0
            return self._bno._report_values[self.feature_id]
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    # This method allows the object to be implicitly converted to the (v1, v2, v3) tuple.
    def __iter__(self):
        """Allows direct unpacking: x, y, z = bno.acceleration"""
        try:
            x, y, z, _, _ = self._bno._report_values[self.feature_id]
            return iter((x, y, z))
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None


class SensorFeature4:
    """
    4-tuple reports with optional metadata or optional full.
    bno.quaternion is really 5-tuple, but few need est angle we treat it as 4-tuple
    FUTURE: Explore if estimated angle and how to expose it for advanced users
    bno.geomagnetic_quaternion is really 5-tuple, but few need est angle, so we treat it as 4-tuple
    """
    __slots__ = ("_bno", "feature_id")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id

    def enable(self, hertz=None):
        """Method to enable the feature with a given report rate (hertz)."""
        return self._bno.enable_feature(self.feature_id, hertz)

    # --- Properties to retrieve the latest data from the BNO instance ---
    @property
    def meta(self):
        """Returns (accuracy, timestamp_ms)."""
        try:
            _, _, _, _, acc, ts = self._bno._report_values[self.feature_id]
            return acc, ts
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    @property
    def full(self):
        """Returns (v1, v2, v3, real, accuracy, timestamp_ms)."""
        try:
            self._bno._unread_report_count[self.feature_id] = 0
            return self._bno._report_values[self.feature_id]
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    @property
    def euler(self):
        """Returns converted Euler 3-tuple plus  accuracy and timestamp_ms."""
        try:
            x, y, z, real, _, _ = self._bno._report_values[self.feature_id]
            roll, pitch, yaw = euler_conversion(x, y, z, real)
            return roll, pitch, yaw
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    @property
    def euler_full(self):
        """Returns converted Euler 3-tuple plus  accuracy and timestamp_ms."""
        try:
            x, y, z, real, acc, ts = self._bno._report_values[self.feature_id]
            roll, pitch, yaw = euler_conversion(x, y, z, real)
            return roll, pitch, yaw, acc, ts
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    # This method allows the object to be implicitly converted to the (v1, v2, v3, v4) tuple.
    def __iter__(self):
        """Allows direct unpacking: x, y, z, real = bno.quaternion"""
        try:
            x, y, z, real, _, _ = self._bno._report_values[self.feature_id]
            return iter((x, y, z, real))
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    # class SensorReading5:
    """
    5-tuple reading with optional metadata and optional full.
    FUTURE: process ARVR rotation and full quaternion implementation wth estimated angle
    will have to use different scalar ror estimated angle!
    """


class RawSensorFeature:
    """
    raw reports Feature manager: raw_acceleration & raw_magnetic (data_count=3) and raw_gyro (data_count=4).
    """
    __slots__ = ("_bno", "feature_id", "data_count")

    def __init__(self, bno_instance, feature_id, data_count):
        self._bno = bno_instance
        self.feature_id = feature_id
        self.data_count = data_count

    def enable(self, hertz=None):
        """Method to enable the feature with a given report rate (hertz)."""
        return self._bno.enable_feature(self.feature_id, hertz)

    def __iter__(self):
        try:
            # Slices the list/tuple from the beginning up to the number of sensor values
            sensor_values = self._bno._report_values[self.feature_id][:self.data_count]
            return iter(sensor_values)
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()"
            ) from None


class BNO08X:
    """Library for the BNO08x IMUs from Ceva - Hillcrest Laboratories
        Main flow of Sensor read:
        1. user calls bno.acceleration - reads sensor data/metadata from _report_values[report_id]
        2. _process_available_packets() - a packet can have multiple reports (0xfb, 0x01, 0x01)
        3. _handle_packet() - splits packets into multiple reports, FIFO new overwrites old
        4. _process_report()
            a. processes sensor reports directly
                i. sensor results & metadata (accuracy & timestamp) put into _report_values[report_id]
                ii. update count in _unread_report_count[report_id] += 1
            b. _handle_control_report - timestamps and vatious command responses/reports

        Note: timestamp is ms(millisec) since 1st BNO08x interrupt, which is close to sensor power up.
    """

    def __init__(self, _interface, reset_pin=None, int_pin=None, cs_pin=None, wake_pin=None, debug=False) -> None:

        self._reset_pin = reset_pin
        self._int_pin = int_pin
        self._wake_pin = wake_pin
        self._cs_pin = cs_pin
        self._interface = _interface
        self._debug = debug

        # set int_pin interrupt, Active-low interrupt → falling edge
        self._int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._on_interrupt)

        self._dbg(f"********** __init__ on {self._interface} Interface *************\n")
        self._data_buffer: bytearray = bytearray(DATA_BUFFER_SIZE)
        self._data_buffer_memoryview = memoryview(self._data_buffer)
        self._command_buffer: bytearray = bytearray(12)
        self._packet_slices = []
        self.last_interrupt_us = -1  # used to signal first interrupt
        self.prev_interrupt_us = -1
        self.ms_at_interrupt = 0
        self._data_available = False
        self._sensor_epoch_ms = 0.0
        self._last_base_timestamp_us = 0

        # track RX(inbound) and TX(outbound) sequence numbers one per channel, one per direction
        self._rx_sequence_number: list[int] = [0, 0, 0, 0, 0, 0]
        self._tx_sequence_number: list[int] = [0, 0, 0, 0, 0, 0]

        self._dcd_saved_at: float = -1
        self._me_calibration_started_at: float = -1.0
        self._calibration_started = False
        self._product_id_received = False
        self._reset_mismatch = False  # if reset_pin set make sure hardware reset done, else pin bad

        self._report_periods_dictionary_us = {}

        self._features = {}  # Create feature objects once
        self._report_values = {}  # most recent sensor values, only if enabled
        self._unread_report_count = {}  # reports received but not yet read by user

        self.reset_sensor()

        # if no int_pin at this point, raise error because we know lot commmunication has occured
        if self.ms_at_interrupt == 0:
            raise RuntimeError("No int_pin signals, check int_pin wiring")

        # send channel 0 BNO_CHANNEL_SHTP_COMMAND, send _COMMAND_ADVERTISE (0) to get more sensor info with debug=True
        self._dbg("*** Request _COMMAND_ADVERTISE")
        data = bytearray(2)
        data[0] = _COMMAND_ADVERTISE
        data[1] = 0
        self._wake_signal()
        self._send_packet(SHTP_CHAN_COMMAND, data)
        # wait for response, ignore packets until _GET_FEATURE_RESPONSE (0xfc)
        try:
            packet_response = self._wait_for_packet(
                SHTP_CHAN_COMMAND,
                _COMMAND_ADVERTISE,
                timeout=_FEATURE_ENABLE_TIMEOUT,
                handle_packet=False
            )
            self._dbg(f" Received Packet *************{packet_response}")

        except RuntimeError:
            raise RuntimeError(f"BNO08X init: not able to get Command Advertise: {hex(_COMMAND_ADVERTISE)}")

        self._dbg("********** End __init__ *************\n")

    def _on_interrupt(self, pin):
        """
        Interrupt handler for active-low int_pin (H_INTN). At first interrupt captures host & bno time
         * self._sensor_epoch_ms set to 0.0 ms at first interrupt
         * self._epoch_start_ms set to ticks_ms() at first interrupt
         * self.last_interrupt_ms set for timebase calculations
        """
        if self.last_interrupt_us == -1:
            self._epoch_start_ms = ticks_ms()  # self._sensor_epoch_ms = 0.0  set in __init__
        self.prev_interrupt_us = self.last_interrupt_us
        self.last_interrupt_us = ticks_us()
        self.ms_at_interrupt = ticks_ms()
        self._data_available = True

    def reset_sensor(self):
        """ After power on, sensor seems to require hard reset, soft reset may be useful after hard reset """
        if self._reset_pin:
            self._hard_reset()
            reset_type = "Hard"
        else:
            self._soft_reset()
            reset_type = "Soft"

        if self._check_id() and not self._reset_mismatch:
            self._dbg(f"*** {reset_type} reset successful, acknowledged with 0xF8 response")
            sleep_ms(100)  # allow SHTP time to settle
            self._tx_sequence_number = [0, 0, 0, 0, 0, 0]
            self._rx_sequence_number = [0, 0, 0, 0, 0, 0]
            return

        if self._reset_mismatch:
            raise RuntimeError("Reset cause mismatch; check reset_pin wiring")

        # sleep_ms(600)  # is this excessive?

        raise RuntimeError(f"Failed to get valid Product ID Response (0xf8) with {reset_type} reset")

    ############ USER VISIBLE REPORT FUNCTIONS ###########################

    def update_sensors(self):
        return self._parse_packets()

    # 3-Tuple Sensor Reports + accuracy + timestamp
    @property
    def linear_acceleration(self):
        """Current linear acceleration values on the X, Y, and Z axes in meters per second squared"""
        report_id = BNO_REPORT_LINEAR_ACCELERATION
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature3(self, report_id)
        return self._features[report_id]

    @property
    def acceleration(self):
        report_id = BNO_REPORT_ACCELEROMETER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature3(self, report_id)
        return self._features[report_id]

    @property
    def gravity(self):
        """gravity vector in the X, Y, and Z components axes in meters per second squared"""
        report_id = BNO_REPORT_GRAVITY
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature3(self, report_id)
        return self._features[report_id]

    @property
    def gyro(self):
        """Gyro's rotation measurements on the X, Y, and Z axes in radians per second"""
        report_id = BNO_REPORT_GYROSCOPE
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature3(self, report_id)
        return self._features[report_id]

    @property
    def magnetic(self):
        """current magnetic field measurements on the X, Y, and Z axes"""
        report_id = BNO_REPORT_MAGNETOMETER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature3(self, report_id)
        return self._features[report_id]

    # 4-Tuple Sensor Reports + accuracy + timestamp
    @property
    def quaternion(self):
        """A quaternion representing the current rotation vector"""
        report_id = BNO_REPORT_ROTATION_VECTOR
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature4(self, report_id)
        return self._features[report_id]

    @property
    def geomagnetic_quaternion(self):
        """A quaternion representing the current geomagnetic rotation vector"""
        report_id = BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature4(self, report_id)
        return self._features[report_id]

    @property
    def game_quaternion(self):
        """A quaternion representing the current rotation vector with no specific reference for heading,
        while roll and pitch are referenced against gravity. To prevent sudden jumps in heading due to corrections,
        the `game_quaternion` property is not corrected using the magnetometer. Drift is expected ! """
        report_id = BNO_REPORT_GAME_ROTATION_VECTOR
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature4(self, report_id)
        return self._features[report_id]

    # raw reports to not support .full
    @property
    def raw_acceleration(self):
        """raw acceleration from registers 3 data value and a raw timestamp"""
        report_id = BNO_REPORT_RAW_ACCELEROMETER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = RawSensorFeature(self, report_id, data_count=4)
        return self._features[report_id]

    @property
    def raw_gyro(self):
        """ raw gyroscope from registers 3 data value, only sensor that reports Celsius, and a raw timestamp"""
        report_id = BNO_REPORT_RAW_GYROSCOPE
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = RawSensorFeature(self, report_id, data_count=5)
        return self._features[report_id]

    @property
    def raw_magnetic(self):
        """ raw magnetic from registers 3 data value and a raw timestamp"""
        report_id = BNO_REPORT_RAW_MAGNETOMETER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = RawSensorFeature(self, report_id, data_count=4)
        return self._features[report_id]

    # Other Sensor Reports
    @property
    def steps(self):
        """ The number of steps detected since the sensor was initialized"""
        report_id = BNO_REPORT_STEP_COUNTER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature1(self, report_id)
        return self._features[report_id]

    @property
    def stability_classifier(self):
        """Returns the sensor's assessment of its current stability:
        * "Unknown" - unable to classify the current stability
        * "On Table" - at rest on a stable surface with very little vibration
        * "Stationary" - below the stable threshold but stable duration has not been met, requires gyro calibration
        * "Stable" - met the stable threshold and duration requirements.
        * "In motion" - sensor is moving.
        """
        report_id = BNO_REPORT_STABILITY_CLASSIFIER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature1(self, report_id)
        return self._features[report_id]

    @property
    def activity_classifier(self):
        """Returns the sensor's assessment of the activity:
        * "Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still"
        * "Tilting", "Walking"     "Running",    "On Stairs"
        """
        report_id = BNO_REPORT_ACTIVITY_CLASSIFIER
        if report_id not in self._features:  # If object not found, create and cache it
            self._features[report_id] = SensorFeature2(self, report_id)
        return self._features[report_id]

    # =============  User helper functions  =============
    def bno_start_diff(self, ticks: int) -> int:
        """ Return milliseconds difference between ticks and sensor startup """
        return ticks_diff(ticks, self._epoch_start_ms)

    @staticmethod
    def euler_conversion(i, j, k, r):
        """
        Converts quaternion values to Euler angles to degrees.
        Uses common aerospace/robotics convention (XYZ rotation order: roll-pitch-yaw).
        """
        jsqr = j * j
        t0 = 2.0 * (r * i + j * k)
        t1 = 1.0 - 2.0 * (i * i + jsqr)
        roll = degrees(atan2(t0, t1))

        t2 = 2.0 * (r * j - k * i)
        t2 = max(-1.0, min(1.0, t2))
        pitch = degrees(asin(t2))

        t3 = 2.0 * (r * k + i * j)
        t4 = 1.0 - 2.0 * (jsqr + k * k)
        yaw = degrees(atan2(t3, t4))

        return roll, pitch, yaw

    @staticmethod
    def degree_conversion(x, y, z):
        """ Converts gyro rad/s to degree/sec """
        return x * 57.2957795, y * 57.2957795, z * 57.2957795

    # ======== Motion Engine (ME) Tare and Calibration (manual) ========

    def tare(self, axis=0x07, basis=None) -> int:
        """
        Tare the sensor
           axis 0x07 (Z,Y,X). Re-orient all motion outputs (accel, gyro, mag, &rotation vectors)
           axis 0x04 (Z-only). changes the heading, but not the tilt

        Rotation Vector or Geomagnetic Rotation Vector will reorient all motion outputs.
           0: quaternion
           1: game_quaternion
           2: geomagnetic_quaternion
           3: Gyro-Integrated Rotation Vector  (not implemented)
           4: ARVR-Stabilized Rotation Vector (not implemented)
           5: ARVR-Stabilized Game Rotation Vector  (not implemented)
        """
        # encode rotation vector to be tared
        if basis > 2:
            raise ValueError(f"Unknown Tare Basis Report ID: {basis}")

        self._dbg(f"TARE: using {hex(basis)=} on {axis=}...")
        self._send_me_command(_ME_TARE_COMMAND,
                              [
                                  _ME_TARE_NOW,  # Perform Tare Now
                                  axis,
                                  basis,  # rotation vector (quaternion) to be tared
                                  0, 0, 0, 0, 0, 0,  # 6-11 Reserved
                              ]
                              )
        return axis, basis

    def clear_tare(self):
        """ Clear the Tare data to flash. """
        self._dbg(f"TARE: Clear Tare...")
        self._send_me_command(_ME_TARE_COMMAND,
                              [
                                  _ME_TARE_SET_REORIENTATION,  # reorientate
                                  0, 0, 0, 0, 0, 0, 0, 0, ]  # 1-8 Reserved
                              )
        return

    def tare_reorientation(self, i, j, k, r):
        """
        Send quaternion reorientation for tare. Can use any of the 3 quaternions.
        Used to set orientation of sensor for example of sensor pcb is mounted vertically
        you can use this to set left edge down, and the other directions.
        """
        # Convert floats to int16 using Q14 fixed-point
        qi = int(i * (1 << 14))
        qj = int(j * (1 << 14))
        qk = int(k * (1 << 14))
        qr = int(r * (1 << 14))

        payload = pack("<hhhh", qi, qj, qk, qr)
        self._dbg(f"TARE: q_int = {(qi, qj, qk, qr)}")
        params = [_ME_TARE_SET_REORIENTATION] + list(payload)
        self._send_me_command(_ME_TARE_COMMAND, params)
        return

    def save_tare_data(self):
        """Save the Tare data to flash"""
        self._dbg(f"TARE Persist data to flash...")
        self._send_me_command(_ME_TARE_COMMAND,
                              [_ME_PERSIST_TARE, 0, 0, 0, 0, 0, 0, 0, 0,]  # 0: command, 1-8 Reserved
                              )
        return

    def begin_calibration(self) -> int:
        """
        Request manual calibration.  6.4.6.1 SH-2: Command Request to configure the ME calibration for
        accelerometer, gyro and magnetometer giving the ability to control when calibration is performed.
        """
        self._send_me_command(_ME_CALIBRATE_COMMAND,
                              [
                                  1,  # calibrate accel
                                  1,  # calibrate gyro
                                  1,  # calibrate mag
                                  _ME_CAL_CONFIG,
                                  0,  # calibrate planar acceleration
                                  0,  # 'on_table' calibration
                                  0, 0, 0, ]  # reserved
                              )
        self._calibration_started = False
        return

    def calibration_status(self) -> int:
        """
        Check if fequest for manual calibration accepted by sensor
        Wait until calibration ready, Send request for status command, wait for response.
        """
        self._send_me_command(_ME_CALIBRATE_COMMAND, [0, 0, 0, _ME_GET_CAL, 0, 0, 0, 0, 0, ])
        return self._calibration_started

    def _send_me_command(self, me_type, me_command) -> None:
        self._dbg(f" ME Command {me_type}: {me_command=}")
        start_time = ticks_ms()
        local_buffer = self._command_buffer
        _insert_command_request_report(
            me_type,
            self._command_buffer,
            self._tx_sequence_number[SHTP_CHAN_CONTROL],
            me_command,
        )
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, local_buffer)

        # change timeout to checking flag for ME Calbiration Response 6.4.6.3 SH-2
        while _elapsed_sec(start_time) < _DEFAULT_TIMEOUT:
            self._parse_packets()
            if self._me_calibration_started_at > start_time:
                break

    def save_calibration_data(self) -> None:
        """ Save the self-calibration data uwing DCD save command"""
        start_time = ticks_ms()
        local_buffer = bytearray(12)
        _insert_command_request_report(
            _SAVE_DCD_COMMAND,
            local_buffer,
            self._tx_sequence_number[SHTP_CHAN_CONTROL],
        )
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, local_buffer)
        while _elapsed_sec(start_time) < _DEFAULT_TIMEOUT:
            self._parse_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")

    #     ############### private/helper methods ###############

    def _parse_packets(self) -> int:
        """ Parse packets and handle reports while data-ready is active."""
        processed_count = 0
        max_packets = _MAX_PACKET_PROCESS
        end_time = ticks_ms() + 10  #10ms guard time
        report_length_map = _REPORT_LENGTHS

        while self._data_ready and processed_count < max_packets:
            if processed_count > 0 and ticks_diff(ticks_ms(), end_time) >= 0:
                self._dbg("_parse_packets processing timed out, 10ms")
                break

            packet = self._read_packet(wait=False)
            if packet is None:
                break

            packet_sh2 = packet.packet_sh2
            data_length = len(packet_sh2)
            if data_length > 0 and packet_sh2[0] == 0x00:
                processed_count += 1
                continue

            # --- START INLINED _handle_packet ---
            next_byte_index = _SHTP_HEADER_LEN  # Payload after the 4-byte SHTP header
            while next_byte_index < data_length:
                report_id = packet_sh2[next_byte_index]
                required_bytes = report_length_map.get(report_id, 0)

                if required_bytes == 0:
                    self._dbg(f"UNSUPPORTED Report ID {hex(report_id)} - SKIPPING ONE BYTE")
                    next_byte_index += 1
                    continue

                unprocessed_byte_count = data_length - next_byte_index
                if unprocessed_byte_count < required_bytes:
                    self._dbg(f"UNSUPPORTED truncated Packet ERROR: {unprocessed_byte_count} bytes")
                    break

                report_view = packet_sh2[next_byte_index: next_byte_index + required_bytes]
                self._process_report(report_id, report_view)

                next_byte_index += required_bytes
            # --- END INLINED _handle_packet  ---

        return processed_count


    def _handle_packet(self, packet):
        """
        Splits a packet into multiple reports and process them in FIFO order.
        NOTE: **** this code is also inlined in _parse_packets for efficiency ****
        """
        data_length = len(packet.packet_sh2)
        next_byte_index = _SHTP_HEADER_LEN  # offset to skip over SHTP header
        report_count = 0

        while next_byte_index < data_length:
            report_id = packet.packet_sh2[next_byte_index]
            required_bytes = _REPORT_LENGTHS.get(report_id, 0)

            if required_bytes == 0:
                self._dbg(f"UNSUPPORTED Report ID {hex(report_id)} at offset {next_byte_index}, ** SKIPPING ONE BYTE")
                next_byte_index += 1
                continue

            unprocessed_byte_count = data_length - next_byte_index
            if unprocessed_byte_count < required_bytes:
                self._dbg(f"Unprocessable truncated batch ERROR: {unprocessed_byte_count} bytes")
                break

            report_view = packet.packet_sh2[next_byte_index: next_byte_index + required_bytes]
            self._process_report(report_id, report_view)
            report_count += 1
            next_byte_index += required_bytes

        # * commented out self._dbg in time critical loops
        # self._dbg(f"HANDLING {report_count} PACKET{'S' if report_count > 1 else ''}...")
        
    def _wait_for_packet(self, channel, report_id=None, timeout=0.5, handle_packet=True):
        """ Wait for a specifc packet to be received on channel, ignore others """
        start_time = ticks_ms()
        while _elapsed_sec(start_time) < timeout:
            try:
                packet = self._read_packet(wait=False)
            except PacketError:
                sleep_ms(1)
                continue

            if packet is not None:
                if packet.channel == channel and (report_id is None or report_id == packet.report_id):
                    if handle_packet:
                        self._handle_packet(packet)

                    return packet

            sleep_ms(1)

        raise RuntimeError(
            f"Timed out waiting for packet on channel {channel} with ReportID {report_id} after {timeout}s"
        )


    def _handle_control_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Handle control reports. Handle time-critical Timestamp methods first
        :param report_id: report ID
        :param report_bytes: portion of packet for report
        :return:
        """
        # Base Timestamp (0xfb)
        if report_id == _BASE_TIMESTAMP:
            self._last_base_timestamp_us = unpack_from("<I", report_bytes, 1)[0] * 100

            # remove self._dbg from time critical operations
            # self._dbg(f"Base Timestamp (0xfb): {self._last_base_timestamp_us} usec")
            return

        # Timestamp Rebase (0xfa), see this when _BASE_TIMESTAMP wraps so use this instead
        if report_id == _TIMESTAMP_REBASE:
            self._last_base_timestamp_us = unpack_from("<I", report_bytes, 1)[0] * 100

            # remove self._dbg from time critical operations
            # self._dbg(f"Timestamp Rebase (0xfa): {self._last_base_timestamp_us} usec")
            return

        # Feature response (0xfc) - This report issued when feature is enabled
        if report_id == _GET_FEATURE_RESPONSE:
            _report_id, feature_report_id = unpack_from("<BB", report_bytes)
            self._report_values[feature_report_id] = _INITIAL_REPORTS.get(feature_report_id, (0.0, 0.0, 0.0, 0, 0.0))
            self._unread_report_count[feature_report_id] = 0
            return

        # Command Response (0xF1) - ME and DCD
        if report_id == _COMMAND_RESPONSE:
            self._handle_command_response(report_bytes)
            return

        # Product ID Response (0xf8)
        if report_id == _REPORT_PRODUCT_ID_RESPONSE:
            reset_cause = report_bytes[1]
            sw_major = report_bytes[2]
            sw_minor = report_bytes[3]
            sw_part_number = unpack_from("<I", report_bytes, 4)[0]
            sw_build_number = unpack_from("<I", report_bytes, 8)[0]
            sw_patch = unpack_from("<H", report_bytes, 12)[0]
            self._dbg("Product ID Response (0xf8):")
            self._dbg(f"*** Last Reset Cause: {reset_cause} = {_RESET_CAUSE_STRING[reset_cause]}")
            self._dbg(f"*** Part Number: {sw_part_number}")
            self._dbg(f"*** Software Version: {sw_major}.{sw_minor}.{sw_patch}")
            self._dbg(f"\tBuild: {sw_build_number}\n")

            # only first Product ID Response report has reset cause, reset_pin should reset_cause=4
            if not self._product_id_received:
                if reset_cause != 4:
                    self._reset_mismatch = True
                    self._dbg(f"Expected 4 for Reset Cause with reset_pin, got {reset_cause}")
            self._product_id_received = True
            return

    def _handle_command_response(self, report_bytes: bytearray) -> None:
        report_body = unpack_from("<BBBBB", report_bytes)
        response = unpack_from("<BBBBBBBBBBB", report_bytes, 5)
        (_report_id, _seq_number, command, _command_seq_number, _response_seq_number,) = report_body

        cal_status, accel_en, gyro_en, mag_en, planar_en, table_en, *_reserved = response

        if command == _ME_CALIBRATE_COMMAND and cal_status == 0:
            self._me_calibration_started_at = ticks_ms()
            self._calibration_started = True
            self._dbg(f"Ready to start calibration at {ticks_ms()=}")

        elif command == _SAVE_DCD_COMMAND:
            self._dbg(f"DCD Save calibration sucess. Status is {cal_status}")
            if cal_status == _COMMAND_STATUS_SUCCESS:
                self._dcd_saved_at = ticks_ms()
            else:
                raise RuntimeError(f"Unable to save calibration data, status={cal_status}")

    def _process_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Process reports both sensor and control reports
        Extracted accuracy and delay from sensor report (100usec ticks)
        Multiple reports are processed in the order they appear in the packet buffer.
        Last sensor report's value over-write previous in this packet, ex: self._report_values[report_id],
        """
        # handle typical sensor reports first
        if 0x01 <= report_id <= 0x09:
            # uctypes-based sensor reports, Parses 3-tuple, 4-tuple, and 5-tuple
            scalar, count = _SENSOR_SCALING[report_id]

            if count == 3:
                v1, v2, v3 = unpack_from("<hhh", report_bytes,  4)
                sensor_data = (v1 * scalar, v2 * scalar, v3 * scalar)
            elif count == 4:
                v1, v2, v3, v4 = unpack_from("<hhhh", report_bytes,  4)
                sensor_data = (v1 * scalar, v2 * scalar, v3 * scalar, v4 * scalar)
            elif count == 5:  # Note likely different scalar when implement count=5
                v1, v2, v3, v4, e1 = unpack_from("<hhhhh", report_bytes,  4)
                sensor_data = (v1 * scalar, v2 * scalar, v3 * scalar, v4 * scalar, e1)
                raise NotImplementedError(f"5-tuple Reports not supported yet,likely different scalar for e1.")
            else:
                raise ValueError("...", )

            # Extract accuracy from byte2 low bits, Extract delay from byte2 & byte3(14 bits)
            byte2 = report_bytes[2]
            accuracy = byte2 & 0x03
            delay_raw = ((byte2 >> 2) << 8) | report_bytes[3]
            delay_ms = delay_raw * 0.1  # notice delay_ms is a float, we have 0.1ms accuracy

            # remove self._dbg from time critical operations
            # self._dbg(f"Report: {_REPORTS_DICTIONARY[report_id]}\nData: {sensor_data}, {accuracy=}, {delay_ms=}")

            # host-based msec timestamps: ints loose 0.1ms accuracy, 32-bit FP don't have enough significant digits
            # self._sensor_ms = self.last_interrupt_ms - self._last_base_timestamp_us + delay_ms
            # Best to use msec in float since first sensor interrupt
            self._sensor_ms = ticks_diff(self.ms_at_interrupt,
                                         self._epoch_start_ms) - self._last_base_timestamp_us * 0.001 + delay_ms
            self._report_values[report_id] = sensor_data + (accuracy, self._sensor_ms)
            self._unread_report_count[report_id] += 1
            return

        #  **** Handle all control reports, here because some are time-critical
        if report_id >= 0xF0:
            self._handle_control_report(report_id, report_bytes)
            return

        if report_id == BNO_REPORT_STEP_COUNTER:
            self._report_values[report_id] = unpack_from("<H", report_bytes, 8)[0]
            return

        if report_id == BNO_REPORT_STABILITY_CLASSIFIER:
            classification_bitfield = unpack_from("<B", report_bytes, 4)[0]
            stability_classification = ["Unknown", "On Table", "Stationary", "Stable", "In motion"][
                classification_bitfield]
            self._report_values[BNO_REPORT_STABILITY_CLASSIFIER] = stability_classification
            return

        # Activitity Classifier in SH-2 (6.5.36)
        if report_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            end_and_page, most_likely_idx = unpack_from("<BB", report_bytes, 4)
            page = end_and_page & 0x7F
            raw_conf = unpack_from("<9B", report_bytes, 6)
            confidence = page * 10 + raw_conf[most_likely_idx]
            activity_name = ACTIVITIES[most_likely_idx]
            self._report_values[BNO_REPORT_ACTIVITY_CLASSIFIER] = activity_name, confidence
            return

        # Raw accelerometer and Raw Magnetometer: returns 4-tuple: x, y, z, and time_stamp
        # timestamp (int) units in internal time?
        if report_id in (BNO_REPORT_RAW_ACCELEROMETER, BNO_REPORT_RAW_MAGNETOMETER):
            x, y, z = unpack_from("<HHH", report_bytes, 4)
            time_stamp = unpack_from("<I", report_bytes, 12)[0]
            sensor_data = (x, y, z, time_stamp)
            self._report_values[report_id] = sensor_data
            return

        # Raw gyroscope: returns 5-tuple: x, y, z, Celsius, and time_stamp
        # timestamp (int) units in internal time?, Celsius in float
        if report_id == BNO_REPORT_RAW_GYROSCOPE:
            raw_x, raw_y, raw_z, temp_int, time_stamp = unpack_from("<HHHhI", report_bytes, 4)
            celsius = (temp_int * 0.5) + 23.0
            sensor_data = (raw_x, raw_y, raw_z, celsius, time_stamp)
            self._report_values[report_id] = sensor_data
            return

        if 0x28 <= report_id <= 0x29:
            # FUTURE: add two ARVR reports (4-tuple and 5-Tuple)
            raise NotImplementedError(f"ARVR Reports ({hex(report_id)}) is not supported yet.")

        # All other reports skipped, noted with self._dbg
        self._dbg(f"_process_report: ({hex(report_id)}) not supported.")
        self._dbg(f"report: {report_bytes}")
        raise NotImplementedError(
            f"Un-implemented Report ({hex(report_id)=}) not supported yet.\n report: {report_bytes}")

    # Enable given feature/sensor report on BNO08x (See SH2 6.5.4)
    def enable_feature(self, feature_id, freq=None):
        """
        Enable sensor features for bno08x, set report period in usec (not msec)
        Called recursively because raw reports require non-raw reports to be enabled
        On Channel (0x02), send _SET_FEATURE_COMMAND (0xfb) with feature id and requested period
        On Channel (0x02), await GET_FEATURE_RESPONSE (0xfc) with actual eabled period
        :returns frequency (float) actual frequency the sensor will attempt to use
        """
        self._dbg(f"Send SET_FEATURE_COMMAND (0xfd) to enable FEATURE ID: {hex(feature_id)}")
        set_feature_report = bytearray(17)
        set_feature_report[0] = _SET_FEATURE_COMMAND
        set_feature_report[1] = feature_id

        if freq is None:
            freq = DEFAULT_REPORT_FREQ[feature_id]

        if freq != 0:
            requested_interval = int(1_000_000 / freq)
        else:
            requested_interval = 0  # effectively turns of reports? but feature still enabled

        pack_into("<I", set_feature_report, 5, requested_interval)

        if feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            pack_into("<I", set_feature_report, 13, _ENABLED_ACTIVITIES)

        # raw sensor rate cannot be higher than the underlying sensor rate
        feature_dependency = _RAW_REPORTS.get(feature_id, None)
        if feature_dependency and feature_dependency not in self._report_values:
            self._dbg(f" Feature dependency detected, now also enable...")
            self._dbg(f"{_REPORTS_DICTIONARY[feature_dependency]} {hex(feature_dependency)}")
            self.enable_feature(feature_dependency, freq)

        # send request _SET_FEATURE_COMMAND (0xfb) with requested period
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, set_feature_report)

        # wait for response, ignore packets until _GET_FEATURE_RESPONSE (0xfc)
        try:
            report_bytes = self._wait_for_packet(SHTP_CHAN_CONTROL, _GET_FEATURE_RESPONSE,
                                                 timeout=_FEATURE_ENABLE_TIMEOUT)
            packet_response = report_bytes.packet_sh2
            fid = packet_response[5]
            report_interval = unpack_from("<I", packet_response, 9)[0]
            self._report_values[feature_id] = _INITIAL_REPORTS.get(feature_id, (0.0, 0.0, 0.0, 0, 0))
            self._report_periods_dictionary_us[feature_id] = report_interval
            self._dbg(f"Report enabled: {_REPORTS_DICTIONARY[fid]}: {hex(fid)}")
            self._dbg(f" Initial tuple={self._report_values}")
            self._dbg(f" Requested Interval: {requested_interval / 1000.0:.1f} ms")
            self._dbg(f" Actual    Interval: {report_interval / 1000.0:.1f} ms\n")
            return 1_000_000. / report_interval

        except RuntimeError:
            raise RuntimeError(f"BNO08X: enable_feature: not able to enable feature: {hex(feature_id)}")

    def print_report_period(self):
        """ Print out heading and row for each report enabled. """
        if not self._report_periods_dictionary_us:
            print("No BNO08x sensors are currently enabled.")
            return

        print(f"Enabled Report Periods and Hz:")
        for feature_id in self._report_periods_dictionary_us.keys():
            period_ms = self._report_periods_dictionary_us[feature_id] / 1000.0
            print(f"\t{_REPORTS_DICTIONARY[feature_id]}\t{period_ms:.1f} ms, {1_000 / period_ms:.1f} Hz")

    def _check_id(self):
        """
        Send Product ID request then process packets until _COMMAND_RESPONSE (0xf1) received.
        Then handle packets, the first 0xf8 indicates reset success and last reset cause.
        Total of 4 0xf8 is normal (last 3 0xf8 will have reset causes = 0).
        
        Sometimes a 0xf8 will be set alone and not headed up with a _COMMAND_RESPONSE (0xf1),
        that case is also acceptable
        """
        self._dbg("********** Check ID **********")
        if getattr(self, "_product_id_received", False):
            return True

        # On channel 2 send PRODUCT_ID_REQUEST (0xf9)
        data = bytearray(2)
        data[0] = _REPORT_PRODUCT_ID_REQUEST
        data[1] = 0
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, data)

        # On channel 2, read/skip packets until _COMMAND_RESPONSE (0xf1)
        start_time = ticks_ms()
        while _elapsed_sec(start_time) < 3.0:
            try:
                packet = self._read_packet(wait=True)
                reportid = packet.report_id
                if packet is None:
                    continue
                if packet.channel != SHTP_CHAN_CONTROL:
                    self._dbg("_check_id skipping above packet\n")
                    continue
                if packet.byte_count - _SHTP_HEADER_LEN == 0:
                    continue
                if packet.report_id == _COMMAND_RESPONSE:
                    # Handle packet to process _REPORT_PRODUCT_ID_RESPONSE reports (0xf8)
                    self._handle_packet(packet)
                    break
                if packet.report_id == _REPORT_PRODUCT_ID_RESPONSE:
                    # if report is _REPORT_PRODUCT_ID_RESPONSE reports (0xf8)
                    self._handle_packet(packet)
                    return True
            except (RuntimeError, PacketError):
                continue

        # check if _REPORT_PRODUCT_ID_RESPONSE (0xf8) received
        if getattr(self, "_product_id_received", False):
            return True

        raise RuntimeError(f"Timeout waiting for valid BNO Product ID response, check {self._interface} interface")

    def _dbg(self, *args, **kwargs) -> None:
        if self._debug:
            print("DBG::\t\t", *args, **kwargs)

    def _hard_reset(self) -> None:
        """Hardware reset the sensor to an initial unconfigured state"""
        if not self._reset_pin:
            return

        self._dbg("*** Hard Reset starting...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_us(10)  # sleep_us(1), data sheet say only 10ns required,
        self._reset_pin.value(1)
        sleep_ms(500)  # orig was 10ms, datasheet implies 94 ms required
        self._dbg("*** Hard Reset End, awaiting acknowledgement (0xf8)")

    def _soft_reset(self) -> None:
        """Send the 'reset' command packet on Executable Channel (1), Section 1.3.1 SHTP"""
        self._dbg(f"*** Soft Reset, Channel={SHTP_CHAN_EXE} command={_COMMAND_RESET}, starting...")
        reset_payload = bytearray([_COMMAND_RESET])
        self._wake_signal()
        self._send_packet(SHTP_CHAN_EXE, reset_payload)
        sleep_ms(500)
        start_time = ticks_ms()
        self._dbg("*** Soft Reset End, awaiting acknowledgement (0xf8)")

    def _wake_signal(self):
        """ Wake is only performaed for spi operation  """
        if self._wake_pin is not None:
            self._dbg("WAKE pulse to BNO08x")
            self._wake_pin.value(0)
            sleep_us(500)  # typ 150 usec required in datasheet BNO datasheet Fig 6-=11 Host Int timing SPI
            self._wake_pin.value(1)

    def _send_packet(self, channel, data):
        raise RuntimeError("_send_packet Not implemented in bno08x.py, supplanted by I2C or SPI subclass")

    def _read_packet(self, wait):
        raise RuntimeError("_read_packet Not implemented in bno08x.py, supplanted by I2C or SPI subclass")

    @property
    def _data_ready(self):
        """ Returns True if at least one new interrupt seen """
        return self.last_interrupt_us != self.prev_interrupt_us

    @property
    def _unread_reports_exist(self):
        """True only when processed sensor reports exist"""
        return any(count > 0 for count in self._unread_report_count.values())


# must define alias after BNO08X class, so class SensorReading4 class can use this
euler_conversion = BNO08X.euler_conversion
