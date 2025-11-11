# BNO08X Micropython Driver by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
# - Also inspired by dobodu
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

TODO BRC enabling all activities seems like overkill
TODO: BRC Euler/quaternion implementation
TODO: BRC add TARE
TODO: BRC update RAW_Sensors
# TODO:
# Default Reports Frequencies (Hz) - does this code have report frequencies right?
# Calibrated Acceleration (m/s2)
# Euler Angles (in degrees?)
# CALIBRATION
"""

__version__ = "0.1"
__repo__ = "https://github.com/bradcar/bno08x_i2c_spi_MicroPython"

from math import asin, atan2, degrees
from struct import pack_into, unpack_from

from collections import namedtuple
from micropython import const
from utime import ticks_ms, ticks_diff, sleep_ms

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

_SHTP_REPORT_PRODUCT_ID_RESPONSE = const(0xF8)
_SHTP_REPORT_PRODUCT_ID_REQUEST = const(0xF9)

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

_DEFAULT_REPORT_INTERVAL = const(50000)  # in microseconds = 50ms, 20 MHz
_QUAT_READ_TIMEOUT = 0.500  # timeout in seconds
_PACKET_READ_TIMEOUT = 2.000  # timeout in seconds
_FEATURE_ENABLE_TIMEOUT = 2.0  # timeout in seconds
_DEFAULT_TIMEOUT = 2.0  # timeout in seconds
_BNO08X_CMD_RESET = const(0x01)
_QUAT_Q_POINT = const(14)  # TODO dobodu sets to 0x05 ???
_BNO_HEADER_LEN = const(4)

# Report Frequencies in Hertz
DEFAULT_REPORT_FREQ = 20
AVAIL_REPORT_FREQ = {
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
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),  # For testing
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),  # For testing
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),  # For testing
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
    BNO_REPORT_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    # Gyro is a 5 tuple, Celsius float and int timestamp for last two entry
    BNO_REPORT_RAW_GYROSCOPE: (0, 0, 0, 0.0, 0),
    # Acc & Mag are 4-tuple, int timestamp for last entry
    BNO_REPORT_RAW_ACCELEROMETER: (0, 0, 0, 0),
    BNO_REPORT_RAW_MAGNETOMETER: (0, 0, 0, 0),
    BNO_REPORT_STEP_COUNTER: 0,
}

_ENABLED_ACTIVITIES = 0x1FF  # for ACTIVITY_CLASSIFIER enable 9  activities: 1 bit set for each of 8 activities and 1 Unknown

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
def _parse_sensor_report_data(report_bytes: bytearray) -> tuple[tuple, int]:
    """Parses reports with only 16-bit fields"""
    data_offset = 4  # this may not always be true
    report_id = report_bytes[0]
    scalar, count, _report_length = _AVAIL_SENSOR_REPORTS[report_id]
    results = []

    format_str = "<H" if report_id in _RAW_REPORTS else "<h"

    byte2 = unpack_from("<B", report_bytes, 2)[0]
    byte3 = unpack_from("<B", report_bytes, 3)[0]
    accuracy = byte2 & 0x03

    # Extract delay upper 6 bits (bits 7:2), combine and adjust from 100us units
    delay_upper = (byte2 >> 2) & 0x3F
    delay_raw = (delay_upper << 8) | byte3
    delay_us = delay_raw * 100

    for _offset_idx in range(count):
        total_offset = data_offset + (_offset_idx * 2)
        raw_data = unpack_from(format_str, report_bytes, total_offset)[0]
        scaled_data = raw_data * scalar
        results.append(scaled_data)
    results_tuple = tuple(results)

    return results_tuple, accuracy, delay_us


def _report_length(report_id: int) -> int:
    if report_id < 0xF0:  # it's a sensor report
        return _AVAIL_SENSOR_REPORTS[report_id][2]

    return _REPORT_LENGTHS[report_id]


def _parse_step_counter_report(report_bytes: bytearray) -> int:
    return unpack_from("<H", report_bytes, 8)[0]


def _parse_stability_classifier_report(report_bytes: bytearray) -> str:
    classification_bitfield = unpack_from("<B", report_bytes, 4)[0]
    return ["Unknown", "On Table", "Stationary", "Stable", "In motion"][classification_bitfield]


# Set Feature Command (0xfd) - host to sensor in SH-2 (6.5.4)
# report_id (B), feature_report_id(B), feature_flags (B), change_sensitivity(H),
# report_interval (I), batch_interval_word (I), sensor_specific_configuration_word (I)
def _parse_get_feature_response_report(report_bytes: bytearray):
    return unpack_from("<BBBHIII", report_bytes)


# Personal Activity Classifier (0x1E), in SH-2 (6.5.36)
def _parse_activity_classifier_report(report_bytes: bytearray) -> dict[str, str]:
    activities = [
        "Unknown",
        "In-Vehicle",
        "On-Bicycle",
        "On-Foot",
        "Still",
        "Tilting",
        "Walking",
        "Running",
        "OnStairs",
    ]

    end_and_page_number = unpack_from("<B", report_bytes, 4)[0]
    page_number = end_and_page_number & 0x7F
    most_likely = unpack_from("<B", report_bytes, 5)[0]
    confidences = unpack_from("<BBBBBBBBB", report_bytes, 6)

    classification = {"most_likely": activities[most_likely]}
    for idx, raw_confidence in enumerate(confidences):
        confidence = (10 * page_number) + raw_confidence
        activity_string = activities[idx]
        classification[activity_string] = confidence
    return classification


def _parse_shake_report(report_bytes: bytearray) -> bool:
    shake_bitfield = unpack_from("<H", report_bytes, 4)[0]
    return (shake_bitfield & 0x07) > 0


def _parse_timestamp(buffer: bytearray) -> tuple[int, ...]:
    """Parse the timestamp unit is 100usec tics
        return usecs
    """
    if not (buffer[0] == _BASE_TIMESTAMP or buffer[0] == _TIMESTAMP_REBASE):
        raise AttributeError(f"Wrong report id Timestamp Base or Rebase: {hex(buffer[0])}")
    return unpack_from("<I", buffer, 1)[0] * 100


def _parse_sensor_id(buffer: bytearray) -> tuple[int, ...]:
    """Parse the fields of a product id report"""
    if not buffer[0] == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
        raise AttributeError("Wrong report id for sensor id: %s" % hex(buffer[0]))

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
        raise AttributeError(
            "Command request reports can only have up to 9 arguments but %d were given"
            % len(command_params)
        )
    for _i in range(12):
        buffer[_i] = 0
    buffer[0] = _COMMAND_REQUEST
    buffer[1] = next_sequence_number
    buffer[2] = command
    if command_params is None:
        return

    for idx, param in enumerate(command_params):
        buffer[3 + idx] = param


def _separate_batch(packet, report_slices):
    # get first report id, loop up its report length
    # read that many bytes, parse them
    next_byte_index = 0
    while next_byte_index < packet.header.data_length:
        report_id = packet.data[next_byte_index]
        required_bytes = _report_length(report_id)

        unprocessed_byte_count = packet.header.data_length - next_byte_index

        # handle incomplete remainder
        if unprocessed_byte_count < required_bytes:
            raise RuntimeError("Unprocessable Batch bytes", unprocessed_byte_count)
        # we have enough bytes to read
        # add a slice to the list that was passed in
        report_slice = packet.data[next_byte_index: next_byte_index + required_bytes]

        report_slices.append([report_slice[0], report_slice])
        next_byte_index = next_byte_index + required_bytes


class Packet:
    """A class representing a Hillcrest Laboratory Sensor Hub Transport packet"""

    def __init__(self, packet_bytes: bytearray) -> None:
        self.header = self.header_from_buffer(packet_bytes)
        data_end_index = self.header.data_length + _BNO_HEADER_LEN
        self.data = packet_bytes[_BNO_HEADER_LEN:data_end_index]

    def __str__(self) -> str:
        length = self.header.packet_byte_count
        outstr = "\n\t\t********** Packet *************\n"
        outstr += "DBG::\t\t HEADER:\n"
        outstr += "DBG::\t\t Data Len: %d\n" % self.header.data_length
        outstr += "DBG::\t\t Channel: %s (%d)\n" % (
            channels[self.channel_number],
            self.channel_number,
        )
        if self.channel_number in {
            _BNO_CHANNEL_CONTROL,
            _BNO_CHANNEL_INPUT_SENSOR_REPORTS,
        }:
            if self.report_id in reports:
                outstr += "DBG::\t\t \tReport Type: %s (0x%x)\n" % (
                    reports[self.report_id],
                    self.report_id,
                )
            else:
                outstr += "DBG::\t\t \t** UNKNOWN Report Type **: %s\n" % hex(self.report_id)

            # TODO BRC this is not correct for 0xfb
            #             if self.report_id > 0xF0 and len(self.data) >= 6 and self.data[5] in reports:
            #                 outstr += "DBG::\t\t \tSensor Report Type: %s(%s)\n" % (
            #                     reports[self.data[5]],
            #                     hex(self.data[5]),
            #                 )

            if self.report_id == 0xFC and len(self.data) >= 6 and self.data[1] in reports:
                outstr += "DBG::\t\t \tEnabled Feature: %s(%s)\n" % (
                    reports[self.data[1]],
                    hex(self.data[1]),
                )
        outstr += "DBG::\t\t Sequence number: %s\n" % self.header.sequence_number
        outstr += "\n"
        outstr += "DBG::\t\t Data:"

        for idx, packet_byte in enumerate(self.data[:length]):
            packet_index = idx + 4
            if (packet_index % 4) == 0:
                outstr += f"\nDBG::\t\t[0x{packet_index:02X}] "
            outstr += f"0x{packet_byte:02X} "
        outstr += "\n"
        outstr += "\t\t*******************************\n"

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
        packet_byte_count = unpack_from("<H", packet_bytes, 0)[0]
        packet_byte_count &= ~0x8000
        channel_number = unpack_from("<B", packet_bytes, 2)[0]
        sequence_number = unpack_from("<B", packet_bytes, 3)[0]
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


class BNO08X:
    """Library for the BNO08x IMUs from Hillcrest Laboratories

    :param

    """

    def __init__(self, reset_pin=None, int_pin=None, cs_pin=None, wake_pin=None, debug=False) -> None:

        self._debug = debug
        self._reset_pin = reset_pin
        self._int_pin = int_pin
        self._wake_pin = wake_pin
        self._cs_pin = cs_pin
        self._dbg("********** __init__ *************")
        self._data_buffer: bytearray = bytearray(DATA_BUFFER_SIZE)
        self._data_buffer_memoryview = memoryview(self._data_buffer)
        self._command_buffer: bytearray = bytearray(12)
        self._packet_slices = []
        self._timestamp_us = 0
        self._rebase_us = 0

        # TODO: this is wrong there should be one per channel per direction
        # This is true — the SHTP spec requires independent sequence counters for:
        # Fix: Replace with:
        # self._sequence_numbers = {ch: {"tx": 0, "rx": 0} for ch in range(6)}
        # then update: _get_report_seq_id() and _update_sequence_number()
        self._sequence_number: list[int] = [0, 0, 0, 0, 0, 0]
        self._two_ended_sequence_numbers: dict[int, int] = {}
        self._dcd_saved_at: float = -1
        self._me_calibration_started_at: float = -1.0
        self._calibration_complete = False
        self._magnetometer_accuracy = 0
        self._wait_for_initialize = True
        self._init_complete = False
        self._id_read = False
        self._quaternion_euler_vector = BNO_REPORT_GAME_ROTATION_VECTOR  # default can change with set_quaternion_euler
        # dictionary of most recent values from each sensor report
        self._report_values = {}
        self._report_periods_dictionary_us = {}
        self.initialize()
        self._dbg("********** End __init__ *************\n")

    def initialize(self):
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
    @property
    def magnetic(self):
        """A tuple of the current magnetic field measurements on the X, Y, and Z axes"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_MAGNETOMETER]
        except KeyError:
            raise RuntimeError("No Magnetometer report found, is it enabled?") from None

    @property
    def quaternion(self):
        """A quaternion representing the current rotation vector"""
        self._process_available_packets()
        try:
            # TODO BRC understand
            return self._report_values[self._quaternion_euler_vector]
        except KeyError:
            raise RuntimeError("No quaternion report found, is it enabled?") from None

    @property
    def euler(self):
        # A 3-tuple representing the current Roll, Tilt, and Yaw euler angle in degree
        self._process_available_packets()
        try:
            q = self._report_values[self._quaternion_euler_vector]
        except KeyError:
            raise RuntimeError("No quaternion report found, is it enabled?") from None

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

        return roll, tilt, yaw

    @property
    def geomagnetic_quaternion(self):
        """A quaternion representing the current geomagnetic rotation vector"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError("No geomagnetic quaternion report found, is it enabled?") from None

    @property
    def game_quaternion(self):
        """A quaternion representing the current rotation vector expressed as a quaternion with no
        specific reference for heading, while roll and pitch are referenced against gravity. To
        prevent sudden jumps in heading due to corrections, the `game_quaternion` property is not
        corrected using the magnetometer. Some drift is expected"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_GAME_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError("No game quaternion report found, is it enabled?") from None

    @property
    def steps(self):
        """The number of steps detected since the sensor was initialized"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_STEP_COUNTER]
        except KeyError:
            raise RuntimeError("No steps report found, is it enabled?") from None

    @property
    def linear_acceleration(self):
        """A tuple representing the current linear acceleration values on the X, Y, and Z
        axes in meters per second squared"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_LINEAR_ACCELERATION]
        except KeyError:
            raise RuntimeError("No linear acceleration report found, is it enabled?") from None

    @property
    def acceleration(self):
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_ACCELEROMETER]
        except KeyError:
            raise RuntimeError("No acceleration report found, is it enabled?")

    @property
    def gravity(self):
        """A tuple representing the gravity vector in the X, Y, and Z components
        axes in meters per second squared"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_GRAVITY]
        except KeyError:
            raise RuntimeError("No gravity report found, is it enabled?") from None

    @property
    def gyro(self):
        """A tuple representing Gyro's rotation measurements on the X, Y, and Z
        axes in radians per second"""
        self._process_available_packets()
        try:
            return self._report_values[BNO_REPORT_GYROSCOPE]
        except KeyError:
            raise RuntimeError("No gyroscope report found, is it enabled?") from None

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
            raise RuntimeError("No shake report found, is it enabled?") from None

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
            raise RuntimeError("No stability classification report found, is it enabled?") from None

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
            raise RuntimeError("No activity classification report found, is it enabled?") from None

    @property
    def raw_acceleration(self):
        """Returns the sensor's raw, unscaled value from the accelerometer registers"""
        self._process_available_packets()
        try:
            raw_acceleration = self._report_values[BNO_REPORT_RAW_ACCELEROMETER]
            return raw_acceleration
        except KeyError:
            raise RuntimeError("No raw acceleration report found, is it enabled?") from None

    @property
    def raw_gyro(self):
        """Returns the sensor's raw, unscaled value from the gyro registers"""
        self._process_available_packets()
        try:
            raw_gyro = self._report_values[BNO_REPORT_RAW_GYROSCOPE]
            return raw_gyro
        except KeyError:
            raise RuntimeError("No raw gyroscope report found, is it enabled?") from None

    @property
    def raw_magnetic(self):
        """Returns the sensor's raw, unscaled value from the magnetometer registers"""
        self._process_available_packets()
        try:
            raw_magnetic = self._report_values[BNO_REPORT_RAW_MAGNETOMETER]
            return raw_magnetic
        except KeyError:
            raise RuntimeError("No raw magnetic report found, is it enabled?") from None

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
            local_buffer,  # should use self._data_buffer :\ but send_packet doesn't
            self._get_report_seq_id(_COMMAND_REQUEST),
        )
        self._send_packet(_BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(_COMMAND_REQUEST)
        while _elapsed_sec(start_time) < _DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")

    ############### private/helper methods ###############

    def _process_available_packets(self, max_packets: int = 10) -> None:
        """
            Read and handle up to `max_packets` packets while data-ready is active.
            
        Returns: True - if processed_count > 0
        """
        processed_count = 0
        start_time = ticks_ms()

        while self._data_ready and processed_count < max_packets:
            try:
                new_packet = self._read_packet()
            except PacketError:
                # transient read errors should not block
                self._dbg("PacketError in _process_available_packets")
                # small delay prevents hammering SPI if line is unstable
                sleep_ms(2)
                continue

            if new_packet:
                self._handle_packet(new_packet)
                processed_count += 1

            # safety timeout (e.g., stuck DRDY line)
            if ticks_diff(ticks_ms(), start_time) > 50:
                self._dbg("Timeout in _process_available_packets")
                break
        flag = processed_count > 0
        self._dbg(f"_process_available_packets done, {processed_count} packets processed - {flag}")
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

    # update the cached sequence number so we know what to increment from
    # TODO: this is wrong there should be one per channel per direction
    # and apparently per report as well
    def _update_sequence_number(self, new_packet: Packet) -> None:
        channel = new_packet.channel_number
        seq = new_packet.header.sequence_number
        self._sequence_number[channel] = seq

    # Dobodu addressed: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/49
    # Dobodu debugged CircuitPython's issue with  RuntimeError: ('Unprocessable Batch bytes', 2)
    def _handle_packet(self, packet):
        """
        Split a single SPI packet into multiple reports and process them in FIFO order.
        Handles multiple 0xF8 Product ID Response reports correctly.
        """
        self._dbg("HANDLING PACKET...")
        try:
            next_byte_index = 0
            slices = []

            while next_byte_index < packet.header.data_length:
                report_id = packet.data[next_byte_index]
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
                    self._dbg("Unprocessable batch: skipping", unprocessed_byte_count, "bytes")
                    break

                report_slice = packet.data[next_byte_index: next_byte_index + required_bytes]
                slices.append([report_slice[0], report_slice])
                next_byte_index += required_bytes

            # Process in FIFO order
            for report_id, report_bytes in slices:
                self._process_report(report_id, report_bytes)

        except Exception as error:
            self._dbg("Error in _handle_packet:", error)
            self._dbg("Packet bytes:", [hex(b) for b in packet.data[:packet.header.data_length]])
            raise

    def _handle_control_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Handle control reports. %imestamps first return quickly
        :param report_id: report ID
        :param report_bytes: portion of packet for report
        :return:
        """
        # Base Timestamp (0xfb)
        if report_id == _BASE_TIMESTAMP:
            self._timestamp_us = _parse_timestamp(report_bytes)
            self._dbg(f"Base Timestamp (0xfb): {self._timestamp_us} usec")
            return

        # Timestamp Rebase (0xfa)
        if report_id == _TIMESTAMP_REBASE:
            self._rebase_us = _parse_timestamp(report_bytes)
            self._dbg(f"Timestamp Rebase (0xfa): {self._rebase_us} usec")
            return

        # Product ID Response (0xf8)
        if report_id == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
            (reset_cause, sw_part_number, sw_major, sw_minor, sw_patch, sw_build_number,) = _parse_sensor_id(
                report_bytes)
            self._dbg("Product ID Response (0xf8):")
            self._dbg(f"*** Last reset cause: {reset_cause} = {_RESET_CAUSE_STRING[reset_cause]}")
            self._dbg(f"*** Part Number: {sw_part_number}")
            self._dbg(f"*** Software Version: {sw_major}.{sw_minor}.{sw_patch}")
            self._dbg(f"\tBuild: {sw_build_number}")
            self._dbg("")
            self._id_read = True

        # Feature response (0xfc)
        # feature_report_id, feature_flags, change_sensitivity, report_interval
        # batch_interval_word, sensor_specific_configuration_word
        if report_id == _GET_FEATURE_RESPONSE:
            get_feature_report = _parse_get_feature_response_report(report_bytes)
            _report_id, feature_report_id, *_remainder = get_feature_report
            self._report_values[feature_report_id] = _INITIAL_REPORTS.get(feature_report_id, (0.0, 0.0, 0.0))

        # Command Response (0xF1)
        if report_id == _COMMAND_RESPONSE:
            self._handle_command_response(report_bytes)

    def _handle_command_response(self, report_bytes: bytearray) -> None:
        (report_body, response_values) = _parse_command_response(report_bytes)

        (_report_id, _seq_number, command, _command_seq_number, _response_seq_number,) = report_body

        # status, accel_en, gyro_en, mag_en, planar_en, table_en, *_reserved) = response_values
        command_status, *_rest = response_values

        if command == _ME_CALIBRATE and command_status == 0:
            self._me_calibration_started_at = ticks_ms()

        if command == _SAVE_DCD:
            if command_status == 0:
                self._dcd_saved_at = ticks_ms()
            else:
                raise RuntimeError("Unable to save calibration data")

    def _process_report(self, report_id: int, report_bytes: bytearray) -> None:
        """
        Process reports both sensor and control reports
        Extract accuracy and delay from each report (100usec ticks)
        TODO: BRC determine how to expose accuracy and delay to users
        """
        if report_id >= 0xF0:
            self._handle_control_report(report_id, report_bytes)
            return
        self._dbg(f"_process_report: {reports[report_id]}")
        #         if self._debug:
        #             outstr = ""
        #             for idx, packet_byte in enumerate(report_bytes):
        #                 packet_index = idx
        #                 if (packet_index % 4) == 0:
        #                     outstr += f"\nDBG::\t\t[0x{packet_index:02X}] "
        #                 outstr += f"0x{packet_byte:02X} "
        #             self._dbg(outstr)
        #             self._dbg("")
        if self._debug:
            lines = []
            for i in range(0, len(report_bytes), 4):
                chunk = " ".join(f"0x{b:02X}" for b in report_bytes[i:i + 4])
                lines.append(f"DBG::\t\t[0x{i:02X}] {chunk}")
            self._dbg("\n".join(lines))
            self._dbg("")

        if report_id == BNO_REPORT_STEP_COUNTER:
            self._report_values[report_id] = _parse_step_counter_report(report_bytes)
            return

        if report_id == BNO_REPORT_SHAKE_DETECTOR:
            shake_detected = _parse_shake_report(report_bytes)
            # shake not previously detected - auto cleared by 'shake' property
            try:
                if not self._report_values[BNO_REPORT_SHAKE_DETECTOR]:
                    self._report_values[BNO_REPORT_SHAKE_DETECTOR] = shake_detected
            except KeyError:
                pass
            return

        if report_id == BNO_REPORT_STABILITY_CLASSIFIER:
            stability_classification = _parse_stability_classifier_report(report_bytes)
            self._report_values[BNO_REPORT_STABILITY_CLASSIFIER] = stability_classification
            return

        if report_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            activity_classification = _parse_activity_classifier_report(report_bytes)
            self._report_values[BNO_REPORT_ACTIVITY_CLASSIFIER] = activity_classification
            return

        # Raw Magnetometer: returns 4-tuple: x, y, z, and time_stamp
        # Raw accelerometer: returns 4-tuple: x, y, z, and time_stamp
        # time_stamp units in microseconds
        if report_id == BNO_REPORT_RAW_ACCELEROMETER or report_id == BNO_REPORT_RAW_MAGNETOMETER:
            data_offset = 4
            report_id = report_bytes[0]
            scalar, count, _report_length = _AVAIL_SENSOR_REPORTS[report_id]

            results = []
            # get 3 raw accelerometer x,y,z 16-bit values
            for _offset_idx in range(count):
                total_offset = data_offset + (_offset_idx * 2)
                raw_data = unpack_from("<H", report_bytes, total_offset)[0]
                results.append(raw_data)

            # get 32-bit time_stamp from raw accelerometer, time_stamp units in microseconds
            time_stamp = unpack_from("<I", report_bytes, 12)[0]
            results.append(time_stamp)

            sensor_data = tuple(results)
            if self._debug:
                outstr = "\t\t\t\tReading for %s %s Time_stamp %u" % (_REPORTS_DICTIONARY[report_id], str(sensor_data),
                                                                      time_stamp)
                print(outstr)

            # TODO: FIXME; Sensor reports are batched in a LIFO which means that multiple reports
            # for the same type will end with the oldest/last being kept and the other
            # newer reports thrown away
            self._report_values[report_id] = sensor_data
            return

        # Raw gyroscope: returns 5-tuple: x, y, z, celsius, and time_stamp
        # time_stamp units in microseconds
        # Celsius float units in celsius
        if report_id == BNO_REPORT_RAW_GYROSCOPE:
            data_offset = 4
            report_id = report_bytes[0]
            scalar, count, _report_length = _AVAIL_SENSOR_REPORTS[report_id]

            results = []
            # get 3 raw gyroscope x,y,z 16-bit values
            for _offset_idx in range(count):
                total_offset = data_offset + (_offset_idx * 2)
                raw_data = unpack_from("<H", report_bytes, total_offset)[0]
                results.append(raw_data)

            # get temperature from raw gyroscope
            # Cera support: temp_int is signed 16-bit int, 0.5C/LSB, center offset is 23°C
            temp_int = unpack_from("<h", report_bytes, 10)[0]
            celsius = (temp_int / 2.0) + 23.0
            results.append(celsius)

            # get 32-bit time_stamp from raw gyroscope, time_stamp units in microseconds
            time_stamp = unpack_from("<I", report_bytes, 12)[0]
            results.append(time_stamp)

            sensor_data = tuple(results)
            if self._debug:
                outstr = "\t\t\t\tReading for %s %s Time_stamp %u" % (_REPORTS_DICTIONARY[report_id], str(sensor_data),
                                                                      time_stamp)
                print(outstr)

            # TODO: FIXME; Sensor reports are batched in a LIFO which means that multiple reports
            # for the same type will end with the oldest/last being kept and the other
            # newer reports thrown away
            self._report_values[report_id] = sensor_data
            self._dbg(f"Report: {reports[report_id]}, {sensor_data=}")

            return

        # General Case all other sensors, sensor_data is a 3-tuple
        sensor_data, accuracy, delay_us = _parse_sensor_report_data(report_bytes)
        self._dbg(f"Report: {reports[report_id]}, {sensor_data=}, {accuracy=}, {delay_us=} usec")
        # TODO: BRC only magnetic accuracy can be user visible, but all reports have accuracy
        if report_id == BNO_REPORT_MAGNETOMETER:
            self._magnetometer_accuracy = accuracy

        # TODO: FIXME; Sensor reports are batched in a LIFO which means that multiple reports
        # for the same type will end with the oldest/last being kept and the other
        # newer reports thrown away
        self._report_values[report_id] = sensor_data
        return

    # Enable a given feature of the BNO08x (See Hillcrest 6.5.4)
    def enable_feature(self, feature_id, freq=None):
        """
        Enable Features the bno08x is to return.
        Called recursively since some raw require non-raw to be enabled
        
        send _SET_FEATURE_COMMAND (0xfb) with feature id
        await GET_FEATURE_RESPONSE (0xfc) 
        both are on Channel (0x02)
        """
        self._dbg(f"ENABLING FEATURE ID... {hex(feature_id)}")

        set_feature_report = bytearray(17)
        set_feature_report[0] = _SET_FEATURE_COMMAND
        set_feature_report[1] = feature_id

        if freq is not None:
            AVAIL_REPORT_FREQ[feature_id] = freq
        else:
            AVAIL_REPORT_FREQ[feature_id] = _DEFAULT_REPORT_INTERVAL

        requested_interval = int(1_000_000 / AVAIL_REPORT_FREQ[feature_id])

        pack_into("<I", set_feature_report, 5, requested_interval)

        if feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            pack_into("<I", set_feature_report, 13, _ENABLED_ACTIVITIES)

        feature_dependency = _RAW_REPORTS.get(feature_id, None)
        if feature_dependency and feature_dependency not in self._report_values:
            self._dbg("\tEnabling feature dependency:", feature_dependency)
            self.enable_feature(feature_dependency, AVAIL_REPORT_FREQ[feature_dependency])

        if self._wake_pin is not None:
            self._dbg("Enable feature WAKE Pulse to ensure BNO08x is out of sleep before INT.")
            self._wake_pin.value(0)
            sleep_ms(1)  # lower than 1ms doesn't seem to work
            self._wake_pin.value(1)
            sleep_ms(5)  # 1 ms works, 1 ms sometimes fails

        self._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)

        try:
            # TODO BRC May need to revert to general Packeet handling since previous enabled functions
            # already start sending reports and this may get mixed up
            report_bytes = self._wait_for_packet(_BNO_CHANNEL_CONTROL, _GET_FEATURE_RESPONSE,
                                                 timeout=_FEATURE_ENABLE_TIMEOUT)
            # report_id (B), feature_report_id(B), feature_flags (B), change_sensitivity(H),
            # report_interval (I), batch_interval_word (I), sensor_specific_configuration_word (I)
            _, fid, _, _, report_interval, _, _ = _parse_get_feature_response_report(report_bytes.data)

            self._report_values[feature_id] = _INITIAL_REPORTS.get(feature_id, (0.0, 0.0, 0.0))
            self._report_periods_dictionary_us[feature_id] = report_interval
            self._dbg(f"_report_values={self._report_values}")
            self._dbg(f"Report confirms: Feature ID {hex(fid)=}")
            self._dbg(
                f"Requested: Interval {requested_interval / 1000.0:.1f} ms, {1_000_000 / requested_interval:.1f} Hz")
            self._dbg(f"Actual   : Interval {report_interval / 1000.0:.1f} ms, {1_000_000 / report_interval:.1f} Hz")
            self._dbg(f"Period Dictionary usec ={self._report_periods_dictionary_us}")
            return

        except RuntimeError:
            raise RuntimeError(f"BNO08X: enable_feature: not able to enable feature: {feature_id}")

    # TODO document this for user, put in test code
    def report_period_us(self, feature_id):
        """
        return 
        """
        return self._report_periods_dictionary_us[feature_id]

    def print_report_period(self):
        print(f"Enabled Report Periods:")
        for feature_id in self._report_periods_dictionary_us.keys():
            period_ms = self.report_period_us(feature_id) / 1000.0
            print(f"\t{feature_id}: {_REPORTS_DICTIONARY[feature_id]}, {period_ms:.1f} ms, {1_000 / period_ms:.1f} Hz")

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

    def _get_data(self, index: int, fmt_string: str):
        # index arg is not including header, so add 4 into data buffer
        data_index = index + 4
        return unpack_from(fmt_string, self._data_buffer, data_index)[0]

    @property
    def _data_ready(self) -> None:
        raise RuntimeError("Not implemented")

    def hard_reset(self) -> None:
        """Hardware reset the sensor to an initial unconfigured state"""
        if not self._reset_pin:
            return

        self._dbg("*** Hard Reset start...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_ms(10)  # TODO try sleep_us(1), data sheet say only 10ns required, 
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

    def _increment_report_seq(self, report_id: int) -> None:
        current = self._two_ended_sequence_numbers.get(report_id, 0)
        self._two_ended_sequence_numbers[report_id] = (current + 1) % 256

    def _get_report_seq_id(self, report_id: int) -> int:
        return self._two_ended_sequence_numbers.get(report_id, 0)
