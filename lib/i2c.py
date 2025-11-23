# BNO08X Micropython I2C Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use I2C
"""

from struct import pack_into

from micropython import const
from machine import Pin

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE

_BNO08X_DEFAULT_ADDRESS = const(0x4B)
_BNO08X_BACKUP_ADDRESS = const(0x4A)


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs from CEVA & Hillcrest Laboratories
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
        self._data_buffer[4:4+data_length] = data
        
        if self._debug:
            packet = Packet(self._data_buffer)
            self._dbg("Sending packet:")
            self._dbg(packet)

        mv = memoryview(self._data_buffer)
        self._i2c.writeto(self._bno_i2c_addr, mv[:write_length])

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return self._tx_sequence_number[channel]

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer_memoryview[:4])

        packet_header = Packet.header_from_buffer(self._data_buffer)
        self._dbg("_read_header")
        self._dbg(packet_header)
        return packet_header

    # wait parameter needed for spi.py, but not needed for i2c
    def _read_packet(self, wait=None):
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer_memoryview[:4])

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel = header.channel_number
        sequence_number = header.sequence_number

        self._rx_sequence_number[channel] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")
        packet_byte_count -= 4
        self._dbg(f"{channel=} has {packet_byte_count} bytes available to read")

        self._read(packet_byte_count)

        new_packet = Packet(self._data_buffer)
        self._dbg(f"New Packet: {new_packet}")

        self._update_sequence_number(new_packet)

        return new_packet

    # read header and the additional requested read length
    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")

        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            self._dbg(f"*** ALLOCATION: increased _data_buffer to bytearray({total_read_length})")
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer_memoryview[:total_read_length])

