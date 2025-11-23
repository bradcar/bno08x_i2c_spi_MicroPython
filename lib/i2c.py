# BNO08X Micropython I2C Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
I2C Class that requires BNO08X base Class
"""

from struct import pack_into

import uctypes
from machine import Pin
from micropython import const

from bno08x import BNO08X, Packet, PacketError
from lib.bno08x import DATA_BUFFER_SIZE

_BNO08X_DEFAULT_ADDRESS = const(0x4B)
_BNO08X_BACKUP_ADDRESS = const(0x4A)

_HEADER_STRUCT = {
    "packet_bytes": (uctypes.UINT16 | 0),
    "channel": (uctypes.UINT8 | 2),
    "sequence": (uctypes.UINT8 | 3),
}


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs on I2C

    Args:
        reset_pin: optionl reset to BNO08x
        int_pin: required int_pin that signals BNO08x
        address: I2C address of sensor, which can often be changed with solder blobs on sensor boards
        debug: prints very detailed logs, primarily for driver debug & development.
    """

    def __init__(self, i2c_bus, address=_BNO08X_DEFAULT_ADDRESS, reset_pin=None, int_pin=None, debug=False):
        self._i2c = i2c_bus
        self._debug = debug
        _interface = "I2C"

        # Validate the i2c address
        if address == _BNO08X_DEFAULT_ADDRESS:
            self._dbg("Using default I2C address.")
        elif address == _BNO08X_BACKUP_ADDRESS:
            self._dbg("Using backup I2C address.")
        else:
            raise ValueError(
                f"Invalid I2C address {hex(address)}, "
                f"Must be {hex(_BNO08X_DEFAULT_ADDRESS)} or {hex(_BNO08X_BACKUP_ADDRESS)}"
            )
        self._bno_i2c_addr = address

        if int_pin is None:
            raise RuntimeError("int_pin is required for I2C operation")
        if not isinstance(int_pin, Pin):
            raise TypeError(f"int_pin must be a Pin object, not {type(int_pin)}.")
        self._int = int_pin

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"Reset (RST) pin must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin

        # give the parent constructor (BNO08X.__init__), the right values from BNO08X_I2C
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<HBB", self._data_buffer, 0, write_length, channel, seq)
        self._data_buffer[4:4 + data_length] = data

        if self._debug:
            packet = Packet(self._data_buffer)
            self._dbg("Sending packet:")
            self._dbg(packet)

        mv = memoryview(self._data_buffer)
        self._i2c.writeto(self._bno_i2c_addr, mv[:write_length])

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return self._tx_sequence_number[channel]

    def _read_packet(self, wait=None):
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer_memoryview[:4])

        header = Packet.header_from_buffer(self._data_buffer)
        packet_bytes = header.packet_byte_count
        channel = header.channel_number
        sequence = header.sequence_number

        self._rx_sequence_number[channel] = sequence
        if packet_bytes == 0:
            raise PacketError("No packet available")

        packet_bytes -= 4

        # self._dbg commented out in time critical code
        # self._dbg(f"{channel=} has {packet_bytes} bytes available to read")

        total_read_length = packet_bytes + 4

        if total_read_length > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            # self._dbg commented out in time critical code
            # self._dbg(f"*** ALLOCATION: increased _data_buffer to bytearray({total_read_length})")

        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer_memoryview[:total_read_length])

        new_packet = Packet(self._data_buffer)
        self._update_sequence_number(new_packet)
        # self._dbg commented out in time critical code
        # self._dbg(f"New Packet: {new_packet}")

        return new_packet
