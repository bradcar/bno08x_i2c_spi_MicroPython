# BNO08X Micropython UART Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use UART
"""

from struct import pack_into

from utime import sleep_ms

# Assuming bno08x.py and Packet/PacketError definitions are available
from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE, BNO_CHANNEL_SHTP_COMMAND


class BNO08X_UART(BNO08X):
    """Library for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, uart, reset_pin=None, int_pin=None, wake_pin=None, debug=False):
        self._uart = uart
        self._reset = reset_pin
        self._int = int_pin
        self._wake = wake_pin
        if self._wake:
            self._wake.value(1)

        # Call parent constructor first to initialize self._debug and other base attributes.
        super().__init__(reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=wake_pin, debug=debug)

    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4
        byte_buffer = bytearray(1)

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        # Slicing is identical
        self._data_buffer[4: 4 + data_length] = data

        self._uart.write(b"\x7e")  # start byte
        sleep_ms(1)
        self._uart.write(b"\x01")  # SHTP byte
        sleep_ms(1)

        # writing byte-by-byte with a delay, standard UART prefers large write
        for b in self._data_buffer[0:write_length]:
            byte_buffer[0] = b
            self._uart.write(byte_buffer)
            sleep_ms(1)

        sleep_ms(1)
        # end byte
        self._uart.write(b"\x7e")

        # print("Sending", [hex(x) for x in self._data_buffer[0:write_length]])

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    def _read_into(self, buf, start=0, end=None):
        if end is None:
            end = len(buf)

        # MicroPython's UART.read(n) returns 'None' or fewer than 'n' bytes if a timeout occurs
        # CircuitPython's version usually blocks until requested bytes are read or timeout.
        # This loop logic handles the byte-by-byte read with control escaping exactly.
        for idx in range(start, end):
            # Blocking read of 1 byte
            data = self._uart.read(1)

            # Assuming a blocking read will get data, & the surrounding logic handles failure
            if data is None:
                raise RuntimeError("Timeout or no data during UART read")

            b = data[0]
            if b == 0x7D:
                # Blocking read of the next byte
                data = self._uart.read(1)
                if data is None:
                    raise RuntimeError("Timeout or no data after escape byte")
                b = data[0]
                b ^= 0x20
            buf[idx] = b
        # print("UART Read buffer: ", [hex(i) for i in buf[start:end]])

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        data = None
        # Loop until start byte (0x7E) is found
        while True:
            # Blocking read of 1 byte
            data = self._uart.read(1)
            self._dbg(f"_read_packet in while: b = {data}")
            if not data:
                # If no data is available (e.g., non-blocking read or timeout), continue loop
                # In MicroPython, a blocking read of 1 may return None on timeout.
                continue
            b = data[0]
            if b == 0x7E:
                break

        # read protocol id
        data = self._uart.read(1)
        self._dbg(f"_read_packet read protocol id: b = {data}")

        if data and data[0] == 0x7E:  # second 0x7e, skip and read again
            data = self._uart.read(1)

        # required protocol ID (0x01)
        if not data or data[0] != 0x01:
            raise RuntimeError("Unhandled UART control SHTP protocol")

        # read header (4 bytes) - this will use the control escape logic
        self._read_into(self._data_buffer, end=4)

        # print("SHTP Header:", [hex(x) for x in self._data_buffer[0:4]])

    def _read_packet(self, wait=None):
        self._read_header()

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel_number, packet_byte_count - 4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            # Recreate buffer if it's too small
            self._data_buffer = bytearray(packet_byte_count)

        self._read_into(self._data_buffer, start=4, end=packet_byte_count)

        # Check for the packet end byte (0x7E)
        data = self._uart.read(1)
        if not data or data[0] != 0x7E:
            raise RuntimeError("Didn't find packet end")

        new_packet = Packet(self._data_buffer)
        self._dbg(f"New Packet: {new_packet}")

        self._update_sequence_number(new_packet)

        return new_packet

    @property
    def _data_ready(self):
        self._dbg(f"_data_ready: {self._uart.any()}")
        return self._uart.any() >= 4

    def soft_reset(self):
        """Reset the sensor to an initial unconfigured state"""
        print("Soft resetting...", end="")

        data = bytearray([0, 1])
        self._send_packet(BNO_CHANNEL_SHTP_COMMAND, data)
        sleep_ms(500)

        # read the SHTP announce command packet response
        while True:
            packet = self._read_packet()
            if packet.channel_number == BNO_CHANNEL_SHTP_COMMAND:
                break

        data = bytearray([1])
        self._send_packet(BNO_CHANNEL_EXE, data)
        sleep_ms(500)
        self._send_packet(BNO_CHANNEL_EXE, data)
        sleep_ms(500)
