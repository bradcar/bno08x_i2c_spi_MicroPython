# BNO08X Micropython I2C Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#

# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
Subclass of `BNO08X` to use I2C
"""

# from adafruit_bus_device import i2c_device

from struct import pack_into

from micropython import const

from . import BNO08X, DATA_BUFFER_SIZE, Packet, PacketError

_BNO08X_DEFAULT_ADDRESS = const(0x4B)


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, i2c_bus, reset_pin=None, address=_BNO08X_DEFAULT_ADDRESS, debug=False):
        # self.bus_device_obj = i2c_device.I2CDevice(i2c_bus, address)
        super().__init__(reset_pin, debug)
        self._i2c = i2c_bus

        # Searching for BNO08x addresses on I2C bus if not specified (preserve original logic)
        if address is None:
            devices = set(self._i2c.scan())
            mpus = devices.intersection(set(_BNO08X_DEFAULT_ADDRESS))
            nb_of_mpus = len(mpus)
            if nb_of_mpus == 0:
                raise ValueError("No BNO08x detected on 12c")
            elif nb_of_mpus == 1:
                self._bno_i2c_addr = mpus.pop()
                self._dbg("BNO08x found at i2c address", hex(self._bno_i2c_addr))
                self._ready = True
            else:
                raise ValueError("Two BNO08x detected on i2d: must specify a device address")
        else:
            self._bno_i2c_addr = address

    def _send_packet(self, channel, data):
        # Start to make the packet header, 2 bytes for size, 1 byte for channel and 1 byte for sequence
        # I2C address is declared in class : self._bno_i2c_addr
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte
        packet = Packet(self._data_buffer)
        self._dbg("BNO08X i2c Sending packet:")
        self._dbg(packet)
        # BRC
        # with self.bus_device_obj as i2c:
        #     i2c.write(self._data_buffer, end=write_length)
        self._i2c.writeto(self._bno_i2c_addr, self._data_buffer[0:write_length])

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    # returns true if available data was read
    # the sensor provides packet length

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        self._dbg("BNO08x i2c READING HEADER...")

        # with self.bus_device_obj as i2c:
        #     i2c.readinto(self._data_buffer, end=4)  # this is expecting a header
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer[0:4])

        packet_header = Packet.header_from_buffer(self._data_buffer)
        self._dbg(packet_header)
        return packet_header

    def _read_packet(self):
        # I2C address is declared in class : self._bno_i2c_addr
        self._dbg("BNO8X i2c READING PACKET...")
        # BRC
        # with self.bus_device_obj as i2c:
        #     i2c.readinto(self._data_buffer, end=4)  # this is expecting a header?
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer[0:4])

        self._dbg("")
        # print(f"SHTP READ packet header: {[hex(x) for x in self._data_buffer[0:4]]}")

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number
        # BRC ???
        data_length = header.data_length

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            self._dbg("SKIPPING NO PACKETS AVAILABLE IN i2c._read_packet")
            raise PacketError("No packet available")
        packet_byte_count -= 4
        self._dbg(
            "channel",
            channel_number,
            "has",
            packet_byte_count,
            "bytes available to read",
        )

        self._read(packet_byte_count)

        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)

        self._update_sequence_number(new_packet)

        return new_packet

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("BNO08x i2c trying to read", requested_read_length, "bytes")
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            self._dbg(
                "!!!!!!!!!!!! ALLOCATION: increased _data_buffer to bytearray(%d) !!!!!!!!!!!!! "
                % total_read_length
            )
        # BRC
        # with self.bus_device_obj as i2c:
        #     i2c.readinto(self._data_buffer, end=total_read_length)
        self._i2c.readfrom_into(self._bno_i2c_addr, self._data_buffer[0:total_read_length])

    @property
    def _data_ready(self):
        header = self._read_header()

        if header.channel_number > 5:
            self._dbg("\tBNO08X channel number out of range:", header.channel_number)
        if header.packet_byte_count == 0x7FFF:
            print("Byte count is 0x7FFF/0xFFFF; Error?")
            if header.sequence_number == 0xFF:
                print("Sequence number is 0xFF; Error?")
            ready = False
        else:
            ready = header.data_length > 0

        self._dbg("\tBNO08X i2c data ready", ready)
        return ready
