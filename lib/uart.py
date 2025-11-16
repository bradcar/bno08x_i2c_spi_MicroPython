# BNO08X Micropython UART Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use UART

To select UART-SHTP, PS1 must be high "1" and PS0/WAKE must be ground "0".
This driver does not support UART-RVC mode. This means for UART operation reqires wake_pin is None, wake_pin=None

1.2.3.1 UART operation: "Bytes sent from the host to the BNO08X must be separated by at least 100us."

1. The H_INTN pin is driven low prior to the initial byte of UART transmission. It will deassert and reassert
between messages. It is used by the host to timestamp the beginning of data transmission.
2. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
reset.

Baud Rate: 3,000,000 baud Time per byte: ~3.3μs, f the BNO08x is sending bytes back-to-back,
the maximum delay between 2 consecutive bytes should be only a few microseconds.
However, the BNO08x might have internal processing delays for assembling a long report. set delay=5ms.
uart = UART(0, baudrate=3_000_000, tx=Pin(12), rx=Pin(13), timeout=5)

6.5.3 Startup timing
The timing for BNO08X startup for I2C and SPI modes uses Reset & Interrupt.
The host may begin communicating with the BNO08X after it has asserted high on INT.
In UART mode, the BNO08X sends an advertisement message when it is ready to communicate.

"""

from struct import pack_into

from utime import sleep_ms, sleep_us

# Assuming bno08x.py and Packet/PacketError definitions are available
from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE


class BNO08X_UART(BNO08X):
    """Library for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, uart, reset_pin=None, int_pin=None, debug=False):
        self._uart = uart
        self._reset = reset_pin
        self._int = int_pin

        # Call parent constructor first to initialize self._debug and other base attributes.
        # wake_pin must be NONE!  wake_pin/PS0 = 0 (gnd)
        super().__init__(reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    def _send_packet(self, channel, data):
        """
        1.2.3.1 UART operation states: "Bytes sent from the host to the BNO08X must be separated by at least 100us."
        """
        data_length = len(data)
        write_length = data_length + 4
        byte_buffer = bytearray(1)

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._tx_sequence_number[channel]
        self._data_buffer[4: 4 + data_length] = data

        self._uart.write(b"\x7e")  # start byte
        sleep_us(110)
        self._uart.write(b"\x01")  # SHTP byte
        sleep_us(110)

        # writing byte-by-byte with a delay, standard UART prefers large write
        for b in self._data_buffer[0:write_length]:
            byte_buffer[0] = b
            self._uart.write(byte_buffer)
            sleep_us(110)

        self._uart.write(b"\x7e")  # end byte

        # print("Sending", [hex(x) for x in self._data_buffer[0:write_length]])

        self._tx_sequence_number[channel] = (self._tx_sequence_number[channel] + 1) % 256
        return self._tx_sequence_number[channel]
    
    # same as CP
    def _read_into(self, buf, start=0, end=None):
        if end is None:
            end = len(buf)

        # print("Avail:", self._uart.in_waiting, "need", end-start)
        for idx in range(start, end):
            data = self._uart.read(1)
            b = data[0]
            if b == 0x7D:  # control escape
                data = self._uart.read(1)
                b = data[0]
                b ^= 0x20
            buf[idx] = b
        # print("UART Read buffer: ", [hex(i) for i in buf[start:end]])
    
    
    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        # try to read initial packet start byte
        data = None
        while True:
            data = self._uart.read(1)
            if not data:
                continue
            b = data[0]
            if b == 0x7E:
                break

        # read protocol id
        data = self._uart.read(1)
        if data and data[0] == 0x7E:  # second 0x7e
            data = self._uart.read(1)
        if not data or data[0] != 0x01:
            raise RuntimeError("Unhandled UART control SHTP protocol")
        # read header
        self._read_into(self._data_buffer, end=4)
        # print("SHTP Header:", [hex(x) for x in self._data_buffer[0:4]])


    def _read_packet(self, wait=None):
        self._read_header()
#         print(f"rp _d_b: {self._data_buffer[:16]}")

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel = header.channel_number
        sequence_number = header.sequence_number
        
        # Check channel validity (copied from your original code)
        if channel >= len(self._rx_sequence_number):
            print(f"!!! WARNING: Received unexpected {channel=} {hex(channel)=}. Discarding packet.")
            print(f"{self._data_buffer[:16]=}")
            # Read and discard the end byte to clear the buffer for the next packet
            self._uart.read(1) 
            raise PacketError(f"Invalid channel number: {channel}")

        self._rx_sequence_number[channel] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel, packet_byte_count - 4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(packet_byte_count)

        # skip 4 header bytes since they've already been read
        self._read_into(self._data_buffer, start=4, end=packet_byte_count)

        # print("Packet: ", [hex(i) for i in self._data_buffer[0:packet_byte_count]])

        data = self._uart.read(1)
        b = data[0]
        if b != 0x7E:
            raise RuntimeError("Didn't find packet end")

        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)

        self._update_sequence_number(new_packet)

        return new_packet

    @property
    def _data_ready(self):
        self._dbg(f"_data_ready: {self._uart.any()}")
        return self._uart.any() >= 4

    # UART must have it's own hard/soft resets BNO_CHANNEL_SHTP_COMMAND = 0x00) in main class but used as constants here
    def soft_reset(self):
        """Reset the sensor to an initial unconfigured state"""
        print("Soft resetting...", end="")

        data = bytearray([0, 1])
        self._send_packet(0x00, data)
        sleep_ms(500)

        # read the SHTP announce command packet response
        while True:
            packet = self._read_packet()
            if packet.channel_number == 0x00:
                break

        # reset TX sequence numbers
        self._tx_sequence_number = [0, 0, 0, 0, 0, 0] # Reset ALL TX sequences to 0
        self._dbg("End Soft RESET in UART ")


    def hard_reset(self) -> None:
        """Hardware reset the sensor to an initial unconfigured state"""
        if not self._reset_pin:
            return

        self._dbg("*** Hard Reset in UART start...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_ms(10)  # TODO try sleep_us(1), data sheet say only 10ns required,
        self._reset_pin.value(1)
        sleep_ms(500)  # datasheet implies 94 ms needed, 200ms has sporatic errors, increasing to 500
        
        # read the SHTP announce command packet response
        while True:
            try: 
                packet = self._read_packet()
                if packet.channel_number == 0x00:
                    break
            except PacketError:
                # Add a small delay to prevent rapid polling if the sensor is slow to respond.
                sleep_ms(20) 
                continue # Safely retry reading the packet

        # reset TX sequence numbers
        self._tx_sequence_number = [0, 0, 0, 0, 0, 0] # Reset ALL TX sequences to 0
        self._dbg("*** Hard Reset End in UART")
