# BNO08X Micropython SPI Interface by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use SPI
"""
from struct import pack_into

from utime import ticks_ms, sleep_ms

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE, _elapsed_sec


class BNO08X_SPI(BNO08X):
    """Library for the BNO08x IMUs on SPI

    Args:
        spi_bus, cspin, intpin, resetpin, baudrate=1_000_000, debug=False
        spi_bus: The SPI bus used with BNO08x
        cspin: CS pin to signal reads or writes
        intpin: to know when BNO08x is ready
        resetpin: to Reset BNO08x
        baudrate: Maximum of 3 MHz, TODO wny does this defaults to 1_000_000
        debug (bool, optional): Enables print statements used for debugging. Defaults to False.
    """

    def __init__(self, spi_bus, cspin, intpin, resetpin, baudrate=1_000_000, debug=False):
        self._spi = spi_bus
        self._cs = cspin
        self._int = intpin
        self._reset = resetpin
        super().__init__(resetpin, debug)

    def hard_reset(self):

        print("Hard resetting...")
        self._reset.value = True  # perform hardware reset
        sleep_ms(10)
        self._reset.value = False
        sleep_ms(10)
        self._reset.value = True
        self._wait_for_int()
        print("Done!")
        self._read_packet()

    def _wait_for_int(self):
        # print("Waiting for INT...", end="")
        start_time = ticks_ms()
        while _elapsed_sec(start_time) < 3.0:
            if not self._int.value:
                break
        else:
            self.hard_reset()
            # raise RuntimeError("Could not wake up")
        # print("OK")

    def soft_reset(self):
        """Reset the sensor to an initial unconfigured state"""
        # print("Soft resetting...", end="")
        # data = bytearray(1)
        # data[0] = 1
        # _seq = self._send_packet(BNO_CHANNEL_EXE, data)
        # time.sleep(0.5)

        for _i in range(3):
            try:
                _packet = self._read_packet()
            except PacketError:
                sleep_ms(100)
        # print("OK!")
        # all is good!

    def _read_into(self, buf, start=0, end=None):
        self._wait_for_int()

        # with self._spi as spi:
        #     spi.readinto(buf, start=start, end=end, write_value=0x00)
        # print("SPI Read buffer (", end-start, "b )", [hex(i) for i in buf[start:end]])
        if end is None:
            end = len(buf)
        self._cs.value(0)
        self._spi.readinto(buf[start:end], 0x00)
        self._cs.value(1)

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        self._wait_for_int()

        # read header
        # with self._spi as spi:
        #     spi.readinto(self._data_buffer, end=4, write_value=0x00)
        self._cs.value(0)
        self._spi.readinto(self._data_buffer, 0x00, 4)
        self._cs.value(1)
        self._dbg("")
        self._dbg("SHTP READ packet header: ", [hex(x) for x in self._data_buffer[0:4]])

    def _read_packet(self):
        self._read_header()
        halfpacket = False

        print([hex(x) for x in self._data_buffer[0:4]])
        if self._data_buffer[1] & 0x80:
            halfpacket = True
        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel_number, packet_byte_count - 4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(packet_byte_count)

        # re-read header bytes since this is going to be a new transaction
        self._read_into(self._data_buffer, start=0, end=packet_byte_count)
        # print("Packet: ", [hex(i) for i in self._data_buffer[0:packet_byte_count]])

        if halfpacket:
            raise PacketError("read partial packet")

        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)
        self._update_sequence_number(new_packet)
        return new_packet

    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        unread_bytes = 0
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            unread_bytes = total_read_length - DATA_BUFFER_SIZE
            total_read_length = DATA_BUFFER_SIZE

        # with self._spi as spi:
        #     spi.readinto(self._data_buffer, end=total_read_length)
        self._cs.value(0)
        self._spi.readinto(self._data_buffer[:total_read_length])
        self._cs.value(1)
        return unread_bytes > 0

    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        self._wait_for_int()
        # with self._spi as spi:
        #     spi.write(self._data_buffer, end=write_length)
        self._cs.value(0)
        self._spi.write(self._data_buffer[:write_length])
        self._cs.value(1)

        self._dbg("Sending: ", [hex(x) for x in self._data_buffer[0:write_length]])
        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    @property
    def _data_ready(self):
        try:
            self._wait_for_int()
            return True
        except RuntimeError:
            return False
