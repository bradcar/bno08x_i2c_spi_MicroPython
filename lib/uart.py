# BNO08X Micropython UART Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use UART

1. The H_INTN pin is driven low prior to the initial byte of UART transmission. It will deassert and reassert
between messages. It is used by the host to timestamp the beginning of data transmission.
2. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
reset.

Pin 5 (PS1) and Pin 6 (PS0/WAKE) are the host interface protocol selection pins. These pins should be tied to
VDDIO and ground respectively to select the UART-SHTP interface.

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
        1.2.3.1 UART operation states:
        "Bytes sent from the host to the BNO08X must be separated by at least 100us."
        """
        data_length = len(data)
        write_length = data_length + 4
        byte_buffer = bytearray(1)

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._tx_sequence_number[channel]
        # Slicing is identical
        self._data_buffer[4: 4 + data_length] = data

        self._uart.write(b"\x7e")  # start byte
        sleep_us(110) #was sleep_ms(1)
        self._uart.write(b"\x01")  # SHTP byte
        sleep_us(110) #was sleep_ms(1)

        # writing byte-by-byte with a delay, standard UART prefers large write
        for b in self._data_buffer[0:write_length]:
            byte_buffer[0] = b
            self._uart.write(byte_buffer)
            sleep_us(110) #was sleep_ms(1)

        # end byte
        self._uart.write(b"\x7e")

        # print("Sending", [hex(x) for x in self._data_buffer[0:write_length]])

        #print(f"MP _send_packet: PRE-UPDATE TX Seq {channel=} - {self._tx_sequence_number[channel]}") # <-- ADD THIS
        self._tx_sequence_number[channel] = (self._tx_sequence_number[channel] + 1) % 256
        #print(f"MP _send_packet: POST-UPDATE TX Seq {channel=} - {self._tx_sequence_number[channel]}") # <-- ADD THIS

        return self._tx_sequence_number[channel]

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
                #print("MP _read_header: NO DATA") # <-- ADD THIS

                # If no data is available (e.g., non-blocking read or timeout), continue loop
                # In MicroPython, a blocking read of 1 may return None on timeout.
                continue
            b = data[0]
            #print(f"MP _read_header: Read {hex(b)}") # <-- ADD THIS

            if b == 0x7E:
                break

        # read protocol id
        data = self._uart.read(1)
        #print(f"MP _read_header: Protocol ID Check: {data}") # <-- ADD THIS

        self._dbg(f"_read_packet read protocol id: b = {data}")

        if data and data[0] == 0x7E:  # second 0x7e, skip and read again
            data = self._uart.read(1)
            #print(f"MP _read_header: Second 0x7E, new ID: {data}") # <-- ADD THIS


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
        channel = header.channel_number
        sequence_number = header.sequence_number
        
        # TODO remove this after debug
        if channel >= len(self._rx_sequence_number):
            print(f"!!! WARNING: Received unexpected {channel=} {hex(channel)=}. Discarding packet.")
            # Read and discard the end byte to clear the buffer for the next packet
            self._uart.read(1) 
            # Use PacketError so the calling function can loop and try again
            raise PacketError(f"Invalid channel number: {channel}")

        self._rx_sequence_number[channel] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")
        
        #print(f" MP _read_packet: START READ: {channel=}, Seq {sequence_number}, {packet_byte_count=}") # <-- ADD THIS
        self._dbg(f"channel {channel} has {packet_byte_count} bytes available to read")

        if packet_byte_count > DATA_BUFFER_SIZE:
            # Recreate buffer if it's too small
            self._data_buffer = bytearray(packet_byte_count)

        self._read_into(self._data_buffer, start=4, end=packet_byte_count)
        #print(f"MP _read_packet: LAST DATA BYTE: {hex(self._data_buffer[packet_byte_count - 1])}") # <-- ADD THIS

        # Check for the packet end byte (0x7E)
        data = self._uart.read(1)
        if not data or data[0] != 0x7E:
            raise RuntimeError("Didn't find packet end")
        #print("MP _read_packet: Found END 0x7E") # <-- ADD THIS


        new_packet = Packet(self._data_buffer)
        self._dbg(f"New Packet: {new_packet}")

        #print(f"MP _read_packet: PRE-UPDATE RX Seq {channel=} {self._rx_sequence_number[channel]}") # <-- ADD THIS
        self._update_sequence_number(new_packet)
        #print(f"MP _read_packet: POST-UPDATE RX Seq {channel=} {self._rx_sequence_number[channel]}") # <-- ADD THIS

        return new_packet


    @property
    def _data_ready(self):
        self._dbg(f"_data_ready: {self._uart.any()}")
        return self._uart.any() >= 4

    # UART must have it's own hard & soft resets
    # BNO_CHANNEL_SHTP_COMMAND = 0x00) defined in main class but used as constants here
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
        sleep_ms(200)  # orig was 10ms, datasheet implies 94 ms required
        
        # read the SHTP announce command packet response
        while True:
            packet = self._read_packet()
            if packet.channel_number == 0x00:
                break

        # reset TX sequence numbers
        self._tx_sequence_number = [0, 0, 0, 0, 0, 0] # Reset ALL TX sequences to 0
        self._dbg("*** Hard Reset End in UART")


