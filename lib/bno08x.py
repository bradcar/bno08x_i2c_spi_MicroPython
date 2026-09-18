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

Data sent for single sensor report:
Quaternion = timebase + sensor report = 23 bytes
* I2C re-reads header (SPI & UART don't) so I2C must read 27 bytes (23+4)

Raw Interface Payload frequencies (freq & any framing)
 - SPI (3_000_000) is 1.2x faster than UART (3_000_000)
 - SPI (3_000_000) is 8.4x faster than I2C (3_000_000).
 - UART (3_000_000) is 6.9x faster than I2C (400_000).

but i2c & spi must also reread header, 1.1739 penalty (27/23) for quaternion
 - SPI is 1.04x faster than UART.  (76.666/73.737 = 1.0397)
 - SPI is 6.9x faster than I2C.  (622.077/73.737 = 8.436)
 - UART is 8.1x faster than I2C.  (622.0773/76.666 = 8.114)

Current best sensor update periods for 3-tuple acceleration:
- spi:   2.0ms (500 Hz) with acc at 2.0ms (500 Hz)
- uart:  2.0ms (500 Hz) with acc at 2.0ms (500 Hz)
- i2c:   2.0ms (500 Hz) with acc at 2.0ms (500 Hz)
Current best sensor update periods for 4-tuple Quaternion:
- spi:   2.5ms (400 Hz) with quaternion at 2.5ms request (400 Hz)
- uart:  2.55ms (393 Hz) with quaternion at 2.5ms request (400 Hz)
- i2c:   2.9ms (344 Hz) with quaternion at 2.5ms request (400 Hz)

At report frequencies shorter than above, the period will increase, likey because the host isn't
keeping up with the sensor and the sensor packages multiple packets together and this library only
returns data for the latest of each package of reports.

TODO: reorg method order
TODO: create continuation code for UART

Possible future projects:
FUTURE: Capture all report data in a multi-package report (without overwrite), provide user all results
FUTURE: explore adding simple 180 degree calibration(0x0c), page 55 SH-2, but will need move request reports
FUTURE: include estimated ange in full quaternion implementation, maybe make new modifier bno.quaternion.est_angle
FUTURE: process two ARVR reports (rotation vector has estimated angle which has a different Q-point)
"""

__version__ = "1.1.0"
__repo__ = "https://github.com/bradcar/bno08x_i2c_spi_MicroPython"

from math import asin, atan2, degrees
from struct import pack_into, unpack_from, pack

import uctypes
from collections import namedtuple
from machine import Pin
from micropython import const
from utime import ticks_ms, ticks_us, ticks_diff, sleep_ms, sleep_us

# Commands
SHTP_CHAN_COMMAND = const(0)  # Advertisement, request & response
SHTP_CHAN_EXE = const(1)  # Soft reset, execute & complete (not acknowledge)
SHTP_CHAN_CONTROL = const(2)  # Reports 0xf1 to 0xfe, request & response
SHTP_CHAN_INPUT = const(3)  # sensor reports 0x01 to 0x2d, data output
SHTP_CHAN_WAKE_INPUT = const(4)  # used by wake-up sensors - Not implemented
BNO_CHAN_GYRO_ROTATION_VECTOR = const(5)  # high-priority head tracking - Not implementd

channels = {
    0x0: "SHTP_CHAN_COMMAND",
    0x1: "SHTP_CHAN_EXE",  # Commands
    0x2: "SHTP_CHAN_CONTROL",  # Sensor report and timestamps (technically command reports)
    0x3: "SHTP_CHAN_INPUT",  # Command reports
    0x4: "SHTP_CHAN_WAKE_INPUT",
    0x5: "BNO_CHAN_GYRO_ROTATION_VECTOR",  # high-speed gyro and rotation
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
_COMMAND_EXE_RESPONSE = const(0x01)

# Sensor Commands & Command reports
_COMMAND_ADVERTISE = const(0x00)  # Request Advertisement command on Chan 0
_ADVERTISE_REPORT = const(0x00)  # Report ID Advertisement command on Chan 0
_COMMAND_EXE_REPORT = const(0x01)  # Report to Acknowledge Command execute
_COMMAND_RESET = const(0x01)  # Soft Reset command on Chan 0

# Advertisement Tag Processors: {tag_id: (name, format, subtract_header_4, clamp_max_1024)}
_tag_dictionary = {0: ("TAG_NULL", 'S'), 1: ("TAG_GUID", '<I'),
                   2: ("Max Cargo Write", '<H'), 3: ("Max Cargo Read", '<H'),
                   4: ("TAG_MAX_TRANSFER_WRITE", '<H'), 5: ("TAG_MAX_TRANSFER_READ", '<H'),
                   6: ("TAG_NORMAL_CHANNEL", '<B'), 7: ("TAG_WAKE_CHANNEL", '<B'),
                   8: ("TAG_APP_NAME", 'S'), 9: ("TAG_CHANNEL_NAME", 'S'),
                   10: ("TAG_ADV_COUNT", '<B'), 0x80: ("SHTP Version", 'S'),
                   0x81: ("Report List, lengths", 'R')
                   }

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
_FEATURE_ENABLE_TIMEOUT_MS = 2000  # 2.0 second timeout for Enable Features
_ME_DCD_TIMEOUT_MS = 2000  # 2.0 second timeout for ME and DCD

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
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: 1000,
}

# pre-calculate the reciprocals
_Q_POINT_14_SCALAR = 2 ** (14 * -1)
_Q_POINT_12_SCALAR = 2 ** (12 * -1)
_Q_POINT_9_SCALAR = 2 ** (9 * -1)
_Q_POINT_8_SCALAR = 2 ** (8 * -1)
_Q_POINT_4_SCALAR = 2 ** (4 * -1)
_Q_POINT_10_SCALAR = 2 ** (10 * -1)

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
    _REPORT_PRODUCT_ID_RESPONSE: 16,  # 0xf8
    _GET_FEATURE_RESPONSE: 17,  # 0xfc
    _BASE_TIMESTAMP: 5,  # 0xfb
    _TIMESTAMP_REBASE: 5,  # 0xfa
}

# Channel 1 Command Reports
_COMMAND_REPORT_LENGTHS = {
    _COMMAND_EXE_RESPONSE: 1,  # 0x01
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
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 5),  # 0x28, note est acc Q_POINT_12?
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4),  # 0x29
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4),  # 2a is angular momentum Q_POINT_10?
    #     BNO_REPORT_MOTION_REQUEST: (1, 1),  # sent to host periodically? 0x2b
    #     BNO_REPORT_OPTICAL_FLOW: (1 ,1),  #  0x2c
    #     BNO_REPORT_DEAD_RECKONING: (1 ,1), #  0x2d
}

# uctypes layout for standard BNO08x sensor reports
_SENSOR_REPORT_LAYOUT = {
    "report_id": 0 | uctypes.UINT8,
    "status": 1 | uctypes.UINT8,
    "byte2": 2 | uctypes.UINT8,  # byte2 contains accuracy+delay high bits
    "byte3": 3 | uctypes.UINT8,  # byte 3 has delay low bits
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
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_STEP_COUNTER: 0,
}

# Activity classifier Initialization
ACTIVITIES = ["Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still", "Tilting", "Walking", "Running", "OnStairs", ]
_ENABLED_ACTIVITIES = 0x1FF  # Enable 9  activities: 1 bit set for each of 8 activities and 1 Unknown

DATA_BUFFER_SIZE = const(284)  # big enough for 1st advertisement 4+280 payload, then max packet <= 256
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


############ Sensor Methods ###########################
class SensorFeature1:
    """ 1-tuple feature manager with methods for enable and reading"""
    __slots__ = ("_bno", "feature_id")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id

    def enable(self, hertz=None):
        return self._bno.enable_feature(self.feature_id, hertz)

    @property
    def updated(self):
        return self._bno._unread_report_count[self.feature_id] > 0

    @property
    def value(self):
        try:
            self._bno._unread_report_count[self.feature_id] = 0
            return self._bno._report_values[self.feature_id]
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None

    def __iter__(self):
        self._bno._unread_report_count[self.feature_id] = 0
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
        return self._bno.enable_feature(self.feature_id, hertz)

    @property
    def updated(self):
        return self._bno._unread_report_count[self.feature_id] > 0

    # convert object to (v1, v2) tuple
    def __iter__(self):
        """Direct unpacking, ex:  x, y = bno.activity_classifier"""
        try:
            val = self._bno._report_values[self.feature_id]
            self._bno._unread_report_count[self.feature_id] = 0
            yield val[0]
            yield val[1]
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None


class SensorFeature3:
    """ 3-tuple feature manager with methods for enable, reading, and metadata."""
    __slots__ = ("_bno", "feature_id", "_values", "_count")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id
        self._values = bno_instance._report_values
        self._count = bno_instance._unread_report_count

    def enable(self, hertz=None):
        if self.feature_id not in self._values:
            self._values[self.feature_id] = None
        return self._bno.enable_feature(self.feature_id, hertz)

    @property
    def updated(self):
        return self._count[self.feature_id] > 0

    @property
    def meta(self):
        """Returns (accuracy, timestamp_ms)."""
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        return val[3], val[4]

    @property
    def full(self):
        """Returns (v1, v2, v3, accuracy, timestamp_ms)."""
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        return val

    def __iter__(self):
        """Direct unpacking, ex: x, y, z = bno.acceleration"""
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        yield val[0]
        yield val[1]
        yield val[2]

    def _raise_not_enabled(self):
        from bno08x import _REPORTS_DICTIONARY
        report_name = _REPORTS_DICTIONARY.get(self.feature_id, "unknown_sensor")
        raise RuntimeError(f"Feature not enabled, use bno.{report_name}.enable()")


class SensorFeature4:
    """
    4-tuple reports with optional metadata or optional full.
    bno.quaternion is really 5-tuple, but few need est angle we treat it as 4-tuple
    FUTURE: Explore if estimated angle and how to expose it for advanced users
    bno.geomagnetic_quaternion is really 5-tuple, but few need est angle, so we treat it as 4-tuple
    """
    __slots__ = ("_bno", "feature_id", "_values", "_count")

    def __init__(self, bno_instance, feature_id):
        self._bno = bno_instance
        self.feature_id = feature_id
        self._values = bno_instance._report_values
        self._count = bno_instance._unread_report_count

    def enable(self, hertz=None):
        if self.feature_id not in self._values:
            self._values[self.feature_id] = None
        return self._bno.enable_feature(self.feature_id, hertz)

    @property
    def updated(self):
        return self._count[self.feature_id] > 0

    @property
    def meta(self):
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        return val[4], val[5]

    @property
    def full(self):
        """Returns (qr, qi, qj, qk, accuracy, timestamp_ms)."""
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        return val

    @property
    def euler(self):
        """Returns converted Euler 3-tuple (Y-P-R) in degrees."""
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        return euler_conversion(val[0], val[1], val[2], val[3])

    @property
    def euler_full(self):
        """
        Returns converted Euler 3-tuple plus  accuracy and timestamp_ms.
        At Quaternion report unpacking the quaternion order can be changed.
        qr = data[0], qi = data[1], qj = data[2],  qk =data[3]
        reminder: BNO SH2 sensor internally orders i,j,k,r but report_update reorders them during unpack
        """
        data = self._values[self.feature_id]
        if data is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        return euler_conversion(data[0], data[1], data[2], data[3]) + (data[4], data[5])

    def __iter__(self):
        val = self._values[self.feature_id]
        if val is None: self._raise_not_enabled()
        self._count[self.feature_id] = 0
        yield val[0]
        yield val[1]
        yield val[2]
        yield val[3]

    def _raise_not_enabled(self):
        from bno08x import _REPORTS_DICTIONARY
        report_name = _REPORTS_DICTIONARY.get(self.feature_id, "unknown_sensor")
        raise RuntimeError(f"Feature not enabled, use bno.{report_name}.enable()")

    # class SensorReading5:
    """
    5-tuple reading with optional metadata and optional full.
    FUTURE: process ARVR rotation and full quaternion implementation wth estimated angle
    will have to use different scalar ror estimated angle!
    """


class RawSensorFeature:
    """ raw reports Feature manager: raw_acceleration & raw_magnetic (data_count=3) and raw_gyro (data_count=4)"""
    __slots__ = ("_bno", "feature_id", "data_count")

    def __init__(self, bno_instance, feature_id, data_count):
        self._bno = bno_instance
        self.feature_id = feature_id
        self.data_count = data_count

    def enable(self, hertz=None):
        return self._bno.enable_feature(self.feature_id, hertz)

    @property
    def updated(self):
        return self._bno._unread_report_count[self.feature_id] > 0

    def __iter__(self):
        try:
            val = self._bno._report_values[self.feature_id]
            self._bno._unread_report_count[self.feature_id] = 0
            for i in range(self.data_count):
                yield val[i]
        except KeyError:
            raise RuntimeError(
                f"Feature not enabled, use bno.{_REPORTS_DICTIONARY[self.feature_id]}.enable()") from None


############ Base Class ###########################
class BNO08X:
    """Library for the BNO08x IMUs from Ceva - Hillcrest Laboratories
        Main flow of Sensor read:
        1. user calls bno.acceleration - reads sensor data/metadata from _report_values[report_id]
        2. _process_available_packets() - a packet can have multiple reports (0xfb, 0x01, 0x01)
           splits packets into multiple reports, FIFO new overwrites old
        4. _process_report()
            a. processes sensor reports directly
                i. sensor results & metadata (accuracy & timestamp) put into _report_values[report_id]
                ii. update count in _unread_report_count[report_id] += 1
            b. _process_control_report - timestamps and various command responses/reports

        Note: timestamp is ms(millisec) since 1st BNO08x interrupt, which is close to sensor power up.
    """

    def __init__(self, _interface, reset_pin=None, int_pin=None, cs_pin=None, wake_pin=None, debug=False) -> None:

        self._reset_pin = reset_pin
        self._int_pin = int_pin
        self._wake_pin = wake_pin
        self._cs_pin = cs_pin
        self._interface = _interface
        self._debug = debug

        # set int_pin first interrupt, Active-low interrupt → falling edge, which sets all others to _fast_interrupt
        self._int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._first_interrupt)

        self._dbg(f"********** __init__ on {self._interface} Interface *************\n")
        self._max_header_plus_cargo = DATA_BUFFER_SIZE  # big enough for 1st advertisement which will reset this to 256
        self._data_buffer: bytearray = bytearray(DATA_BUFFER_SIZE)
        self._data_buffer_memoryview = memoryview(self._data_buffer)
        self._command_buffer: bytearray = bytearray(12)
        self._packet_slices = []
        self.last_interrupt_us = -1  # us at last interrupt
        self.ms_at_interrupt = 0  # ms at last interrupt, us and ms are indepedently clocking/wrapping
        self._sensor_epoch_ms = 0.0
        self._last_base_timestamp_us = 0
        self._new_data_interrupt = False

        # track RX(inbound) and TX(outbound) sequence numbers one per channel, one per direction
        self._rx_sequence_number: list[int] = [0, 0, 0, 0, 0, 0]
        self._tx_sequence_number: list[int] = [0, 0, 0, 0, 0, 0]
        self._advertisement_received = False

        self._dcd_saved_at: float = -1
        self._me_calibration_started_at: float = -1.0
        self._calibration_started = False
        self._product_id_received = False
        self._reset_mismatch = False  # if reset_pin set make sure hardware reset done, else pin bad

        self._features = {}  # Create feature objects once
        self._report_periods_dictionary_us = {}
        self._report_values = [None] * 45  # Stores most recent sensor values, only if enabled
        self._unread_report_count = bytearray(45)  # array, reports received but read by user, 1:45, (0x01 to 0x2d)

        self.reset_sensor()

        # if no int_pin at this point, raise error because we know lot commmunication has occured
        if self.ms_at_interrupt == 0:
            raise RuntimeError("No int_pin signals, check int_pin wiring")

        self._dbg("********** End __init__ *************\n")

    def _first_interrupt(self, pin):
        """
        int_pin Interrupt handler for active-low (H_INTN). At first interrupt captures host & bno time
         * self._epoch_start_ms set to ticks_ms() at first interrupt
         * self.last_interrupt_ms set for timebase calculations
        """
        self.last_interrupt_us = ticks_us()
        self.ms_at_interrupt = ticks_ms()
        self._new_data_interrupt = True
        self._epoch_start_ms = ticks_ms()  # set epoch start ms on first interrupt
        pin.irq(
            handler=self._fast_interrupt)  # initial Hander, after first interrupt Rebind to the fast interrupt handler

    def _fast_interrupt(self, pin):
        """int_pin Interrupt handler for active-low (H_INTN). Other handler _first_interrupt captures host & bno time"""
        self.last_interrupt_us = ticks_us()
        self.ms_at_interrupt = ticks_ms()
        self._new_data_interrupt = True

    def reset_sensor(self):
        """ After power on, sensor requires synchronization before Product ID Request."""
        self._product_id_received = False
        self._reset_mismatch = False

        if self._reset_pin:
            self._hard_reset()
            reset_type = "Hard"
        else:
            self._soft_reset()
            reset_type = "Soft"

        # BNO08x sends an Advertisement (Chan 0) and Reset Complete (Chan 2)
        self._dbg("Wait for Advertisement after reset and Reset Complete")
        sync_start = ticks_ms()
        while not self._advertisement_received and ticks_diff(ticks_ms(), sync_start) < 500:
            if self.update_sensors() > 0:
                sync_start = ticks_ms()
            sleep_ms(10)

        # Verify Reset with Request Product ID
        self._dbg("Verify rest by Sending PRODUCT_ID_REQUEST (0xf9) on channel 2")
        data = bytearray(2)
        data[0] = _REPORT_PRODUCT_ID_REQUEST
        data[1] = 0
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, data)

        # Await Product ID Response
        start = ticks_ms()
        while not self._product_id_received and ticks_diff(ticks_ms(), start) < 3000:
            if self._new_data_interrupt:
                self.update_sensors()

        if self._product_id_received and not self._reset_mismatch:
            self._dbg(f"*** {reset_type} reset success, acknowledged with first Product ID 0xF8\n")
            return

        if self._reset_mismatch:
            raise RuntimeError(f"{reset_type} reset cause mismatch; check reset_pin wiring")

        raise RuntimeError(
            f"{reset_type} reset not acknowledged, check BNO08x wiring, if SPI/BNO used before UART/BNO then power cycle BNO")

    def _packet_decode(self, packet_length, channel, seq, payload):
        """Packet decode for debugging driver for both read & send packets"""
        outstr = "\n\t\t********** Packet *************\n"
        outstr += "DBG::\t\tHeader:\n"
        outstr += f"DBG::\t\t Packet Len: {packet_length} ({hex(packet_length)})\n"
        outstr += f"DBG::\t\t Channel: {channels[channel]} ({hex(channel)})\n"
        outstr += f"DBG::\t\t Sequence: {seq}\n"
        report_id = payload[0]
        if channel == SHTP_CHAN_INPUT:
            report_name = _REPORTS_DICTIONARY.get(report_id)
            outstr += f"DBG::\t\t Report Type: {report_name} ({hex(report_id)})\n"
        if channel == SHTP_CHAN_CONTROL:
            if report_id == 0xFC and packet_length - _SHTP_HEADER_LEN >= 6 and report_id in _REPORTS_DICTIONARY:
                # first report_id (self.data[0]), the report type to be enabled (self.data[1])
                outstr += f"DBG::\t\t Feature Enabled: {_REPORTS_DICTIONARY[payload[1]]} ({hex(payload[1])})\n"
        outstr += "\nDBG::\t\tData:\n"
        payload_length = packet_length - _SHTP_HEADER_LEN
        outstr += f"DBG::\t\t Data Len: {payload_length}"
        for idx in range(payload_length):
            packet_byte = payload[idx]
            if (idx % 4) == 0:
                outstr += f"\nDBG::\t\t[0x{idx:02X}] "
            outstr += f"0x{packet_byte:02X} "
        outstr += "\n\t\t*******************************\n"

        # preliminary decoding of packets
        if packet_length - _SHTP_HEADER_LEN == 15 and channel == SHTP_CHAN_INPUT and report_id == 0xfb:
            outstr += f"DBG::\t\t Report 1: {_REPORTS_DICTIONARY[payload[5]]} ({hex(payload[5])})\n"

        # New Stye Advertisement Response gives sensor information, first time is 284 bytes, when request is 51 bytes
        if packet_length - _SHTP_HEADER_LEN >= 51 and channel == SHTP_CHAN_COMMAND and report_id == _COMMAND_ADVERTISE:
            outstr += "DBG::\t\tNew Style SHTP Advertisement Response (0x00), channel: SHTP_COMMAND (0x0)\n"
            return outstr

        # OLD Stye Advertisement Response of sensor information, parser removed to reduce code size
        if packet_length - _SHTP_HEADER_LEN == 34 and channel == SHTP_CHAN_COMMAND and report_id == _COMMAND_ADVERTISE:
            outstr += "DBG::\t\tOld Style SHTP Advertisement Response (0x00), channel: SHTP_COMMAND (0x0)\n"
            return outstr

        return outstr

    ############ USER VISIBLE REPORT FUNCTIONS ###########################

    @micropython.native
    def update_sensors(self) -> int:
        """
        Reads new packet then Parse packets into multiple reports. Process based on channel
                channel 3: Timebase, rebase and Sensors, added fastpath for timebase followed by sensor reports 
                Channel 2: Command reports (Multiple single reports, ex: F1,F8's)
                Channel 1: Executable (single reports, TODO verify)
                Channel 0: SHTP command (single reports, TODO verify)
                Channel 5: High-speed Gyro (single reports) - Raises UNIMPLEMENTED
        """
        FP_TO_MS = 0.001
        FP_DIV_TEN = 0.1
        SIGN_BIT = 32768
        processed_count = 0
        report_length_map = _REPORT_LENGTHS.get
        scaling_map = _SENSOR_SCALING.get
        report_values = self._report_values
        unread_report_count = self._unread_report_count

        while self._new_data_interrupt or (hasattr(self, "_uart") and self._uart.any() >= 4):
            self._new_data_interrupt = False
            result = self._read_packet(wait=True)
            if result is None:
                break
            payload, channel, data_length = result
            p_mv = memoryview(payload)
            processed_count += 1
            report_index = 0
            report_id = p_mv[0]

            # fast path for timestamp & reports in a single packet, inlined from self._process_report
            if channel == 3 and report_id == _BASE_TIMESTAMP:
                self._last_base_timestamp_us = (p_mv[1] | (p_mv[2] << 8) | (p_mv[3] << 16) | (p_mv[4] << 24)) * 100
                packet_base_ms = ticks_diff(self.ms_at_interrupt, self._epoch_start_ms) - (
                        self._last_base_timestamp_us * FP_TO_MS)
                report_index += 5  # _BASE_TIMESTAMP is 5 bytes

                # native-compiled fast path - test showed it was slower?
                # self._sensor_fast_path(p_mv, data_length, 5, packet_base_ms, report_length_map, scaling_map, report_values, unread_counts)
                p = p_mv
                while report_index < data_length:
                    report_id = p[report_index]
                    required_bytes = report_length_map(report_id, 0)

                    if required_bytes == 0: break

                    if 0x01 <= report_id <= 0x09:
                        scalar, count = scaling_map(report_id, (0, 0))
                        idx = report_index
                        b2 = p[idx + 2]
                        # accuracy = b2 & 0x03
                        ts = packet_base_ms + (((b2 & 0xFC) << 6) | p[idx + 3]) * FP_DIV_TEN
                        # r is temp variable used to prepare for Q-point scaling
                        r = p[idx + 4] | (p[idx + 5] << 8)
                        v1 = (r - ((r & SIGN_BIT) << 1)) * scalar
                        r = p[idx + 6] | (p[idx + 7] << 8)
                        v2 = (r - ((r & SIGN_BIT) << 1)) * scalar
                        r = p[idx + 8] | (p[idx + 9] << 8)
                        v3 = (r - ((r & SIGN_BIT) << 1)) * scalar

                        if count == 3:
                            report_values[report_id] = (v1, v2, v3, b2 & 0x03, ts)
                        else:  # Handle Quaternion V4
                            r = p[idx + 10] | (p[idx + 11] << 8)
                            # Q-point scales the 4 result returned
                            v4 = (r - ((r & SIGN_BIT) << 1)) * scalar
                            # SH-2 BNO INTERNAL DATA STRUCTURE DIFFERENT ORDER !  (qi, qj, qk, qr)
                            # BUT we unpack and store in proper user (qr, qi, qj, qk) ordering
                            report_values[report_id] = (v4, v1, v2, v3, b2 & 0x03, ts)

                        unread_report_count[report_id] += 1
                        report_index += required_bytes
                    else:
                        self._process_report(report_id, p_mv[report_index: report_index + required_bytes])
                        report_index += required_bytes
                continue

            # Split payload into multiple reports and process
            if channel == 3 or channel == 2:
                while report_index < data_length:
                    report_id = p_mv[report_index]
                    required_bytes = report_length_map(report_id, 0)

                    if required_bytes == 0:
                        self._dbg(f"UNSUPPORTED Report ID {hex(report_id)} - SKIPPING ONE BYTE")
                        report_index += 1
                        continue

                    if data_length - report_index < required_bytes:
                        self._dbg(f"UNSUPPORTED truncated packet ERROR: {data_length - report_index} bytes")
                        break

                    self._process_report(report_id, p_mv[report_index: report_index + required_bytes])
                    report_index += required_bytes

            elif channel == 0:  # all reports on channel 5 are single report packets
                self._process_control_report(0x00, p_mv)

            elif channel == 1:  # all reports on channel 5 are single report packets
                self._process_control_report(p_mv[0], p_mv)

            elif channel == 5:  # gyro integrated rotation vector, 14 bytes raw int16, no report ID/status/delay
                SIGN_BIT = 32768
                Q14 = _Q_POINT_14_SCALAR
                Q10 = _Q_POINT_10_SCALAR
                p = p_mv
                # Quaternion i, j, k, real (Q14) — bytes 0-7
                r = p[0] | (p[1] << 8)
                qi = (r - ((r & SIGN_BIT) << 1)) * Q14
                r = p[2] | (p[3] << 8)
                qj = (r - ((r & SIGN_BIT) << 1)) * Q14
                r = p[4] | (p[5] << 8)
                qk = (r - ((r & SIGN_BIT) << 1)) * Q14
                r = p[6] | (p[7] << 8)
                qr = (r - ((r & SIGN_BIT) << 1)) * Q14
                # Angular velocity x, y, z (Q10) — bytes 8-13
                r = p[8] | (p[9] << 8)
                ax = (r - ((r & SIGN_BIT) << 1)) * Q10
                r = p[10] | (p[11] << 8)
                ay = (r - ((r & SIGN_BIT) << 1)) * Q10
                r = p[12] | (p[13] << 8)
                az = (r - ((r & SIGN_BIT) << 1)) * Q10
                ts = ticks_diff(self.ms_at_interrupt, self._epoch_start_ms)
                report_values[BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR] = (qr, qi, qj, qk, ax, ay, az, ts)
                unread_report_count[BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR] += 1

        return processed_count

    # 3-Tuple Sensor Reports + accuracy + timestamp
    @property
    def linear_acceleration(self):
        """Current linear acceleration values on the X, Y, and Z axes in meters per second squared"""
        return self._get_feature(BNO_REPORT_LINEAR_ACCELERATION, SensorFeature3)

    @property
    def acceleration(self):
        return self._get_feature(BNO_REPORT_ACCELEROMETER, SensorFeature3)

    @property
    def gravity(self):
        """gravity vector in the X, Y, and Z components axes in meters per second squared"""
        return self._get_feature(BNO_REPORT_GRAVITY, SensorFeature3)

    @property
    def gyro(self):
        """Gyro's rotation measurements on the X, Y, and Z axes in radians per second"""
        return self._get_feature(BNO_REPORT_GYROSCOPE, SensorFeature3)

    @property
    def magnetic(self):
        """current magnetic field measurements on the X, Y, and Z axes"""
        return self._get_feature(BNO_REPORT_MAGNETOMETER, SensorFeature3)

    # 4-Tuple Sensor Reports + accuracy + timestamp
    @property
    def quaternion(self):
        """A quaternion representing the current rotation vector"""
        return self._get_feature(BNO_REPORT_ROTATION_VECTOR, SensorFeature4)

    @property
    def geomagnetic_quaternion(self):
        """A quaternion representing the current geomagnetic rotation vector"""
        return self._get_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, SensorFeature4)

    @property
    def game_quaternion(self):
        """A quaternion representing the current rotation vector with no specific reference for heading,
        while roll and pitch are referenced against gravity. To prevent sudden jumps in heading due to corrections,
        the `game_quaternion` property is not corrected using the magnetometer. Drift is expected ! """
        return self._get_feature(BNO_REPORT_GAME_ROTATION_VECTOR, SensorFeature4)

    @property
    def gyro_integrated_rotation_vector(self):
        """Gyro Integrated Rotation Vector — quaternion + angular velocity at up to 1000 Hz.
        Channel 5 high-priority report per SH-2 6.5.44.
        Iterate for (qr, qi, qj, qk); use .full for 8-tuple with angular velocity and timestamp."""
        return self._get_feature(BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR, SensorFeature4)

    # raw reports to not support .full
    @property
    def raw_acceleration(self):
        """raw acceleration from registers 3 data value and a raw timestamp"""
        return self._get_feature(BNO_REPORT_RAW_ACCELEROMETER, RawSensorFeature, 4)

    @property
    def raw_gyro(self):
        """ raw gyroscope from registers 3 data value, only sensor that reports Celsius, and a raw timestamp"""
        return self._get_feature(BNO_REPORT_RAW_GYROSCOPE, RawSensorFeature, 5)

    @property
    def raw_magnetic(self):
        """ raw magnetic from registers 3 data value and a raw timestamp"""
        return self._get_feature(BNO_REPORT_RAW_MAGNETOMETER, RawSensorFeature, 4)

    # Other Sensor Reports
    @property
    def steps(self):
        """ The number of steps detected since the sensor was initialized"""
        return self._get_feature(BNO_REPORT_STEP_COUNTER, SensorFeature1)

    @property
    def stability_classifier(self):
        """Returns the sensor's assessment of its current stability:
        * "Unknown" - unable to classify the current stability
        * "On Table" - at rest on a stable surface with very little vibration
        * "Stationary" - below the stable threshold but stable duration has not been met, requires gyro calibration
        * "Stable" - met the stable threshold and duration requirements.
        * "In motion" - sensor is moving.
        """
        return self._get_feature(BNO_REPORT_STABILITY_CLASSIFIER, SensorFeature1)

    @property
    def activity_classifier(self):
        """Returns the sensor's assessment of the activity:
        * "Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still" "Tilting", "Walking", "Running", "On Stairs"
        """
        return self._get_feature(BNO_REPORT_ACTIVITY_CLASSIFIER, SensorFeature2)

    # =============  User helper functions  =============
    def bno_start_diff(self, ticks: int) -> int:
        """ Return milliseconds difference between ticks and sensor startup """
        return ticks_diff(ticks, self._epoch_start_ms)

    @staticmethod
    def euler_conversion(r, i, j, k):
        """
        Converts quaternion to Euler angles (degrees).
        Android ENU Frame, ENU (East-North-Up).
        Z-Y-X sequence (often referred to as Yaw-Pitch-Roll
        """
        # yaw (physical Z, CCW is +, comes from quaternion k)
        yaw = degrees(atan2(
            2.0 * (r * k + i * j),
            1.0 - 2.0 * (j * j + k * k)
        ))

        # roll (physical roll X, comes from quaternion j)
        # negated to match silkscreen on Sparkfun PCB
        roll = -degrees(atan2(
            2.0 * (r * j + i * k),
            1.0 - 2.0 * (j * j + i * i)
        ))

        # pitch (physical pitch Y, comes from quaternion i)
        t2 = 2.0 * (r * i - j * k)
        t2 = max(-1.0, min(1.0, t2))
        pitch = degrees(asin(t2))

        return yaw, pitch, roll

    @staticmethod
    def degree_conversion(x, y, z):
        """ Converts gyro rad/s to degree/sec """
        return x * 57.2957795, y * 57.2957795, z * 57.2957795

    # ======== Motion Engine (ME) Tare and Calibration (manual) ========

    def tare(self, axis=0x07, basis=None) -> int:
        """
        Tare the sensor
           axis 0x07 (Z,Y,X). Re-orient all motion outputs (accel, gyro, mag, & rotation vectors)
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
        # rotation vector (quaternion) to be tared
        self._send_me_command(_ME_TARE_COMMAND,
                              [_ME_TARE_NOW, axis, basis, 0, 0, 0, 0, 0, 0, ]
                              )
        return axis, basis

    def clear_tare(self):
        """ Clear the Tare data in flash. """
        self._dbg(f"TARE: Clear Tare...")
        self._send_me_command(_ME_TARE_COMMAND,
                              [_ME_TARE_SET_REORIENTATION, 0, 0, 0, 0, 0, 0, 0, 0, ]
                              )
        return

    def tare_reorientation(self, qr, qi, qj, qk):
        """
        Tare with any of the 3 quaternions. Set orientation of sensor (es: sensor pcb is vertical)
        you can use this to set left edge down, and the other directions.
        this is called with user ordering (r,i,j,k), but must pack it in BNO ordering (i,j,k,r)
        """
        # Convert floats to int16 using Q14 fixed-point
        r = int(qr * (1 << 14))
        i = int(qi * (1 << 14))
        j = int(qj * (1 << 14))
        k = int(qk * (1 << 14))

        # this called with proper user (qr, qi, qj, qk) ordering
        # BUT: SH-2 BNO REQUIRES DIFFERENT ORDER !  (qi, qj, qk, qr)
        payload = pack("<hhhh", i, j, k, r)
        self._dbg(f"TARE: q_int = {(qr, qi, qj, qk)}")

        params = [_ME_TARE_SET_REORIENTATION] + list(payload)
        self._send_me_command(_ME_TARE_COMMAND, params)
        return

    def save_tare_data(self):
        """Save the Tare data to flash"""
        self._dbg(f"TARE Persist data to flash...")
        self._send_me_command(_ME_TARE_COMMAND,
                              [_ME_PERSIST_TARE, 0, 0, 0, 0, 0, 0, 0, 0, ]  # 0: command, 1-8 Reserved
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
        Check if request for manual calibration accepted by sensor
        Wait until calibration ready, Send request for status command, wait for response.
        """
        self._send_me_command(_ME_CALIBRATE_COMMAND, [0, 0, 0, _ME_GET_CAL, 0, 0, 0, 0, 0, ])
        return self._calibration_started

    def _send_me_command(self, me_type, me_command) -> None:
        self._dbg(f" ME Command {me_type}: {me_command=}")
        start_time = ticks_ms()
        send_packet = self._command_buffer
        self._insert_command_request_report(
            me_type,
            self._command_buffer,
            self._tx_sequence_number[SHTP_CHAN_CONTROL],
            me_command,
        )
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, send_packet)

        # change timeout to checking flag for ME Calbiration Response 6.4.6.3 SH-2
        while ticks_diff(ticks_ms(), start_time) < _ME_DCD_TIMEOUT_MS:
            self.update_sensors()
            if self._me_calibration_started_at > start_time:
                break

    def save_calibration_data(self) -> None:
        """ Save the self-calibration data uwing DCD save command"""
        seq = self._tx_sequence_number[SHTP_CHAN_CONTROL]
        start_time = ticks_ms()
        send_packet = bytearray(12)
        self._insert_command_request_report(_SAVE_DCD_COMMAND, send_packet, seq)

        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, send_packet)

        while ticks_diff(ticks_ms(), start_time) < _ME_DCD_TIMEOUT_MS:
            self.update_sensors()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")

    def _insert_command_request_report(self,
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
        buffer[3:12] = b'\x00' * 9

        if command_params:
            for idx, param in enumerate(command_params):
                buffer[3 + idx] = param

    # ############### private Core Engine methods ###############

    def _get_feature(self, report_id, cls, *args):
        feat = self._features.get(report_id)
        if feat is None:
            feat = self._features[report_id] = cls(self, report_id, *args)
        return feat

    def _process_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Process reports both sensor reports (channel 3) and control reports (channel 2)
        Extracted accuracy and delay from sensor report (100usec ticks)
        Multiple reports are processed in the order they appear in the packet buffer.
        Last sensor report's value over-write previous in this packet, ex: self._report_values[report_id],

        Must call self._process_control_report directly if reports coming from channel 0 or 1, because
        they have two prolematic report ids (0x00, and 0x01 which is same as acceleration below).

        Timestamps are msec since first sensor interrupt in float(FP) with 0.1ms resolution.
        Host synched int or FP32 issues: overflow, wrap to quickly, or lack precision. FP32 has 6-7 significant digits
        Can't use problematic: self._sensor_ms = self.last_interrupt_ms - self._last_base_timestamp_us + delay_ms
        """
        # process typical sensor reports first
        if 0x01 <= report_id <= 0x09:
            scalar, count = _SENSOR_SCALING[report_id]
            r = uctypes.struct(uctypes.addressof(report_bytes), _SENSOR_REPORT_LAYOUT, uctypes.LITTLE_ENDIAN)

            if count == 3:
                sensor_data = (r.v1 * scalar, r.v2 * scalar, r.v3 * scalar)

            # SH-2 BNO INTERNAL DATA STRUCTURE DIFFERENT ORDER !  (qi, qj, qk, qr)
            # BUT we unpack and store in proper user (qr, qi, qj, qk) ordering
            elif count == 4:
                sensor_data = (r.v4 * scalar, r.v1 * scalar, r.v2 * scalar, r.v3 * scalar)
            # future: handle 5-tuple, WARNING 'e1scalar' for e1 will be different
            # elif count == 5:
            #    sensor_data = (r.v4 * scalar, r.v1 * scalar, r.v2 * scalar, r.v3 * scalar, r.e1 * e1scalar)
            else:
                raise ValueError("Invalid sensor data count, 5-tuple not implemented")

            # Extract accuracy from byte2 low bits, Extract delay from byte2 & byte3(14 bits)
            accuracy = r.byte2 & 0x03
            delay_ms = (((r.byte2 >> 2) << 8) | r.byte3) * 0.1  # delay counts with 0.1ms ticks, convert to FP32

            # remove self._dbg from time critical operations
            # self._dbg(f"Report: {_REPORTS_DICTIONARY[report_id]}\nData: {sensor_data}, {accuracy=}, {delay_ms=}")

            self._sensor_ms = ticks_diff(self.ms_at_interrupt,
                                         self._epoch_start_ms) - self._last_base_timestamp_us * 0.001 + delay_ms
            self._report_values[report_id] = sensor_data + (accuracy, self._sensor_ms)
            self._unread_report_count[report_id] += 1
            return

        # Base Timestamp (0xfb)
        if report_id == _BASE_TIMESTAMP:
            self._last_base_timestamp_us = (report_bytes[1] | (report_bytes[2] << 8) | (
                    report_bytes[3] << 16) | (report_bytes[4] << 24)) * 100
            return

        # Timestamp Rebase (0xfa), this sent when _BASE_TIMESTAMP wraps
        if report_id == _TIMESTAMP_REBASE:
            self._last_base_timestamp_us = (report_bytes[1] | (report_bytes[2] << 8) | (
                    report_bytes[3] << 16) | (report_bytes[4] << 24)) * 100
            return

        #  **** Process all control reports, catchall if processing sensor reports
        if report_id >= 0xF0:
            self._process_control_report(report_id, report_bytes)
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

    def _process_control_report(self, report_id: int, report_bytes: bytearray) -> None:
        """ Process control reports. These are only on Channel 0, 1, or 2 """
        # Base Timestamp (0xfb)
        if report_id == _BASE_TIMESTAMP:
            self._last_base_timestamp_us = (report_bytes[1] | (report_bytes[2] << 8) | (
                    report_bytes[3] << 16) | (report_bytes[4] << 24)) * 100
            return

        # Timestamp Rebase (0xfa), this sent when _BASE_TIMESTAMP wraps
        if report_id == _TIMESTAMP_REBASE:
            self._last_base_timestamp_us = (report_bytes[1] | (report_bytes[2] << 8) | (
                    report_bytes[3] << 16) | (report_bytes[4] << 24)) * 100
            return

        # Feature response (0xfc) - This report issued when feature is enabled or updated
        if report_id == _GET_FEATURE_RESPONSE:
            feature_report_id = report_bytes[1]
            self._unread_report_count[feature_report_id] = 0
            self._report_values[feature_report_id] = _INITIAL_REPORTS.get(feature_report_id, (0.0, 0.0, 0.0, 0, 0.0))
            report_interval = unpack_from("<I", report_bytes, 5)[0]
            self._report_periods_dictionary_us[feature_report_id] = report_interval
            self._dbg(f"Enabled Report: {_REPORTS_DICTIONARY[feature_report_id]}: {hex(feature_report_id)}")
            self._dbg(f" Actual Report Interval: {report_interval / 1000.0:.1f} ms")
            self._dbg(f" All Enabled tuples = {self._report_values}\n")
            return

        # Command Response (0xf1) - confirms reset on i2c/spi, ME, and DCD responses
        if report_id == _COMMAND_RESPONSE:
            self._dbg(f"Command response (0xf1)")
            report_body = unpack_from("<BBBBB", report_bytes)
            response = unpack_from("<BBBBBBBBBBB", report_bytes, 5)
            (_report_id, _seq_number, command, _command_seq_number, _response_seq_number,) = report_body
            cal_status, accel_en, gyro_en, mag_en, planar_en, table_en, *_reserved = response

            if command == 4:
                self._dbg("Received: Command to Re-Initialze BNO08x\n")
            elif command == _ME_CALIBRATE_COMMAND and cal_status == 0:
                self._me_calibration_started_at = ticks_ms()
                self._calibration_started = True
                self._dbg(f"Ready to start calibration at {ticks_ms()=}")
            elif command == _SAVE_DCD_COMMAND:
                self._dbg(f"DCD Save calibration sucess. Status is {cal_status}")

                if cal_status == _COMMAND_STATUS_SUCCESS:
                    self._dcd_saved_at = ticks_ms()
                else:
                    raise RuntimeError(f"Unable to save calibration data, status={cal_status}")

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

            # only first Product ID Response report has reset cause, HW reset_cause=4
            if not self._product_id_received:
                if reset_cause != 4:
                    self._reset_mismatch = True
                    self._dbg(f"Expected 4 for Reset Cause with reset_pin, got {reset_cause}")
            self._product_id_received = True
            return

        # Advertisement TLV encoded (Type-Length-Value), wake advertisement 280 byte payload, 2nd is 51 bytes
        if report_id == _ADVERTISE_REPORT:
            self._dbg("Advertisement response on Channel 0x00")
            length = len(report_bytes)

            if length > 2:
                outstr = "\n\t\t Advertisement TLV Decode:\n"
            else:
                outstr = f"\nDBG::\t\t Data Len: {length}"
                for idx, packet_byte in enumerate(report_bytes):
                    if (idx % 4) == 0:
                        outstr += f"\nDBG::\t\t[0x{idx:02X}] "
                    outstr += f"0x{packet_byte:02X} "
                outstr += "\n\t\t*******************************"

            index = 1  # skip the Report ID (0x00) at payload[0]
            while index + 2 <= length:
                tag = report_bytes[index]
                tag_len = report_bytes[index + 1]
                value_index = index + 2
                next_index = value_index + tag_len

                if next_index > length:
                    break

                value = report_bytes[value_index:next_index]

                if tag in _tag_dictionary:
                    name, fmt = _tag_dictionary[tag]
                    if fmt == 'S':
                        decoded_str = bytes(value).decode('utf-8', 'ignore').strip('\x00')
                        s = "" if tag == 0 else f": {decoded_str}"
                        outstr += f"DBG::\t\t {name}{s}\n"
                    elif fmt == 'R':
                        outstr += f"DBG::\t\t {name} (Report List & Lengths):\n"
                        sub_idx = 0
                        while sub_idx < len(value):
                            rep_id = value[sub_idx]
                            rep_len = value[sub_idx + 1]
                            outstr += f"DBG::\t\t   Report 0x{rep_id:02X}, Length: {rep_len}\n"
                            sub_idx += 2
                    else:
                        v = unpack_from(fmt, value, 0)[0]
                        outstr += f"DBG::\t\t {name}: {v}\n"
                    if tag == 2:
                        self._max_header_plus_cargo = v
                else:
                    outstr += f"DBG::\t\t Unknown tag = {tag}\n"

                index = next_index

            self._dbg(f"{outstr}")
            self._advertisement_received = True
            return

        # Command execution report 0x01, on Channel 0x0
        if report_id == _COMMAND_EXE_REPORT:
            self._dbg("Command Execution Received on Channel (0x0)")
            return

    # Enable given feature/sensor report on BNO08x (See SH2 6.5.4)
    def enable_feature(self, feature_id, freq=None):
        """
        Enable sensor features for bno08x, set report period in usec (not msec)
        Called recursively because raw reports require non-raw reports to be enabled
        On Channel (0x02), send _SET_FEATURE_COMMAND (0xfb) with feature id and requested period
        On Channel (0x02), await GET_FEATURE_RESPONSE (0xfc) with actual eabled period
        :returns: frequency (float) actual frequency the sensor will attempt to use
        """
        self._dbg(f"Send SET_FEATURE_COMMAND (0xfd) to enable FEATURE ID: {hex(feature_id)}")
        feature_enable_request = bytearray(17)
        feature_enable_request[0] = _SET_FEATURE_COMMAND
        feature_enable_request[1] = feature_id

        if freq is None:
            freq = DEFAULT_REPORT_FREQ[feature_id]

        if freq != 0:
            requested_interval = int(1_000_000 / freq)
        else:
            requested_interval = 0  # effectively turns of reports? but feature still enabled

        pack_into("<I", feature_enable_request, 5, requested_interval)

        if feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            pack_into("<I", feature_enable_request, 13, _ENABLED_ACTIVITIES)

        # raw sensor rate cannot be higher than the underlying sensor rate
        feature_dependency = _RAW_REPORTS.get(feature_id, None)
        if feature_dependency and feature_dependency not in self._report_values:
            self._dbg(f" Feature dependency detected, now also enable...")
            self._dbg(f"{_REPORTS_DICTIONARY[feature_dependency]} {hex(feature_dependency)}")
            self.enable_feature(feature_dependency, freq)

        self._dbg(f" Requested Interval: {requested_interval / 1000.0:.1f} ms")
        self._wake_signal()
        self._send_packet(SHTP_CHAN_CONTROL, feature_enable_request)

        # clear older entries, if reinitializing
        if feature_id in self._report_periods_dictionary_us:
            del self._report_periods_dictionary_us[feature_id]

        start_time = ticks_ms()
        while feature_id not in self._report_periods_dictionary_us:
            self.update_sensors()
            if ticks_diff(ticks_ms(), start_time) > _FEATURE_ENABLE_TIMEOUT_MS:
                raise RuntimeError(f"BNO08X: Timeout enabling feature: {hex(feature_id)}")

        actual_interval = self._report_periods_dictionary_us[feature_id]
        # Return frequency
        return 1_000_000. / actual_interval if actual_interval > 0 else 0.0

    def print_report_period(self):
        """ Print out heading and row for each report enabled. """
        if not self._report_periods_dictionary_us:
            print("No BNO08x sensors are currently enabled.")
            return

        print(f"Enabled Report Periods and Hz:")
        for feature_id in self._report_periods_dictionary_us.keys():
            period_ms = self._report_periods_dictionary_us[feature_id] / 1000.0
            print(f"\t{_REPORTS_DICTIONARY[feature_id]}\t{period_ms:.1f} ms, {1_000 / period_ms:.1f} Hz")

    def _dbg(self, *args, **kwargs) -> None:
        if self._debug:
            print("DBG::\t\t", *args, **kwargs)

    def _hard_reset(self) -> None:
        """Hardware reset the sensor to an initial unconfigured state"""
        if not self._reset_pin: return
        self._dbg("Hard Reset starting...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_us(10)  # sleep_us(1), data sheet say only 10ns required,
        self._reset_pin.value(1)
        sleep_ms(500)  # orig was 10ms, datasheet implies 94 ms required
        self._dbg("Hard Reset End, awaiting acknowledgement (0xf8)\n")

    def _soft_reset(self) -> None:
        """Send the 'reset' command packet on Executable Channel (1), Section 1.3.1 SHTP"""
        self._dbg(f"Soft Reset, Channel={SHTP_CHAN_EXE} command={_COMMAND_RESET}, starting...")
        reset_payload = bytearray([_COMMAND_RESET])
        self._wake_signal()
        self._send_packet(SHTP_CHAN_EXE, reset_payload)
        sleep_ms(500)
        start_time = ticks_ms()
        self._dbg("Soft Reset End, awaiting acknowledgement (0xf8)")

    def _wake_signal(self):
        """ Wake is only performaed for spi operation  """
        if self._wake_pin is not None:
            self._dbg("WAKE pulse to BNO08x")
            self._wake_pin.value(0)
            sleep_us(500)  # todo typ 150 usec required in datasheet BNO datasheet Fig 6-=11 Host Int timing SPI
            self._wake_pin.value(1)

    def _send_packet(self, channel, data):
        raise RuntimeError("_send_packet Not implemented in bno08x.py, supplanted by I2C or SPI subclass")

    def _read_packet(self, wait):
        raise RuntimeError("_read_packet Not implemented in bno08x.py, supplanted by I2C or SPI subclass")


# must define alias after BNO08X class, so class SensorReading4 class can use this
euler_conversion = BNO08X.euler_conversion
