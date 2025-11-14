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

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE

_BNO08X_DEFAULT_ADDRESS = const(0x4B)


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, i2c_bus, address=_BNO08X_DEFAULT_ADDRESS, reset_pin=None, int_pin=None, debug=False):
        self._i2c = i2c_bus
        #todo should I do the following
        # self._reset = reset_pin
        # self._int = int_pin

        self._bno_i2c_addr = address if address is not None else _BNO08X_DEFAULT_ADDRESS

        # give the parent constructor (BNO08X.__init__), the right values from BNO08X_I2C
        super().__init__(reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    # TODO
    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._rx_sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte
        packet = Packet(self._data_buffer)
        self._dbg("Sending packet:")
        self._dbg(packet)

        self._i2c.writeto(self._bno_i2c_addr, self._data_buffer[:write_length])

        #todo is this right place for rx update? is this rx or tx?
        self._rx_sequence_number[channel] = (self._rx_sequence_number[channel] + 1) % 256
        return self._rx_sequence_number[channel]

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
        self._dbg("_read_packet")

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._rx_sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            self._dbg("TODO Brad? SKIPPING NO PACKETS AVAILABLE IN i2c._read_packet")
            raise PacketError("No packet available")
        packet_byte_count -= 4
        self._dbg(f"channel {channel_number} has {packet_byte_count} bytes available to read")

        self._read(packet_byte_count)

        new_packet = Packet(self._data_buffer)
        self._dbg(f"New Packet: {new_packet}")

        self._update_sequence_number(new_packet)

        return new_packet

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            self._dbg(f"*** ALLOCATION: increased _data_buffer to bytearray({total_read_length})")
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer_memoryview[:total_read_length])

    @property
    def _data_ready(self):
        header = self._read_header()

        if header.channel_number > 5:
            self._dbg("channel number out of range:", header.channel_number)
        if header.packet_byte_count == 0x7FFF:
            print("Byte count is 0x7FFF/0xFFFF; Error?")
            if header.sequence_number == 0xFF:
                print("Sequence number is 0xFF; Error?")
            ready = False
        else:
            ready = header.data_length > 0

        self._dbg("\tdata ready", ready)
        return ready
