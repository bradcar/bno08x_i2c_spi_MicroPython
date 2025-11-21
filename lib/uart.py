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

1.2.3.1 UART operation: "Bytes sent from the host to the BNO08X must be separated by at least 100μs.
Bytes sent from the BNO to the host have no extra spacing."

The INT pin signifies when the UART data is to be sent:
6.5.4 Interrupt timing: In UART-SHTP mode the interrupt is asserted prior to the UART transmission. It is assumed that the host can
always accept data over its UART. The interrupt is asserted approx. 7.7 µs prior to the first bit of UART
transmission. The interrupt will be de-asserted prior to the termination of the UART transmission.

1. The H_INTN pin is driven low prior to the initial byte of UART transmission. It will deassert and reassert
between messages. It is used by the host to timestamp the beginning of data transmission.
2. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
reset.

Baud Rate: 3_000_000 baud (~3.3μs/byte), if the BNO08x is sending bytes back-to-back,
the maximum delay between 2 consecutive bytes should be only a few microseconds.
However, the BNO08x might have internal processing delays for assembling a long report. set delay=5ms.
uart = UART(1, baudrate=3_000_000, tx=Pin(8), rx=Pin(9), timeout=50)
uart = UART(0, baudrate=3_000_000, tx=Pin(12), rx=Pin(13), timeout=50)

6.5.3 Startup timing
The timing for BNO08X startup for I2C and SPI modes uses Reset & Interrupt.
The host may begin communicating with the BNO08X after it has asserted high on INT.
In UART mode, the BNO08X sends an advertisement message when it is ready to communicate.

"""

from struct import pack_into

from utime import sleep_ms, sleep_us, ticks_ms
from machine import Pin

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE, _elapsed_sec


class BNO08X_UART(BNO08X):
    """
    UART-SHTP class for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, uart, reset_pin=None, int_pin=None, debug=False):
        self._uart = uart
        _interface = "UART"
        
        if int_pin is None:
            raise RuntimeError("int_pin is required for SPI operation")
        if not isinstance(int_pin, Pin):
            raise TypeError("int_pin must be a Pin object, not Pin number")
        self._int = int_pin
        
        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"Reset (RST) pin must be a 'machine.Pin' object or None, not {type(rst_pin)}.")
        self._reset = reset_pin

        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

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
        self._data_buffer[4: 4 + data_length] = data

        self._uart.write(b"\x7e")  # start byte
        sleep_us(100)
        self._uart.write(b"\x01")  # SHTP byte
        sleep_us(100)

        # writing byte-by-byte with the specified delay
        for b in self._data_buffer[0:write_length]:
            byte_buffer[0] = b
            self._uart.write(byte_buffer)
            sleep_us(100)

        self._uart.write(b"\x7e")  # end byte

        self._tx_sequence_number[channel] = (self._tx_sequence_number[channel] + 1) % 256
        return self._tx_sequence_number[channel]

    def _read_into(self, buf, start=0, end=None):
        if end is None:
            end = len(buf)

        for idx in range(start, end):
            data = self._uart.read(1)
            b = data[0]
            if b == 0x7D:  # control escape
                data = self._uart.read(1)
                b = data[0]
                b ^= 0x20
            buf[idx] = b

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        data = None
        start_time = ticks_ms()
        while True:
            data = self._uart.read(1)
            if not data:
                if _elapsed_sec(start_time) > 5.0:
                    print("ERROR 5 sec timeout, Check UART pins & sensor wiring (host TX to SCLK, host RX to SDA)")
                    raise RuntimeError("UART read timeout while waiting for 0x7E start byte.")
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

        self._read_into(self._data_buffer, end=4)

    def _read_packet(self, wait=None):
        self._read_header()
        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel = header.channel_number
        sequence_number = header.sequence_number

        # Check channel validity
        if channel >= len(self._rx_sequence_number):
            raise PacketError(f"Invalid channel number: {channel} {self._data_buffer[:4]=}")

        self._rx_sequence_number[channel] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel, packet_byte_count - 4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(packet_byte_count)

        # skip 4 header bytes since they've already been read
        self._read_into(self._data_buffer, start=4, end=packet_byte_count)

        data = self._uart.read(1)
        if not data:
            raise RuntimeError("Timeout while waiting for packet end")

        b = data[0]

        # Check for escape sequence
        if b == 0x7D:
            data = self._uart.read(1)
            if not data:
                raise RuntimeError("Timeout while waiting for escaped end byte")
            b = data[0] ^ 0x20  # Un-escape the byte

        if b != 0x7E:
            raise RuntimeError("Didn't find packet end")

        new_packet = Packet(self._data_buffer)
        self._dbg(f"{new_packet=}")
        self._update_sequence_number(new_packet)

        return new_packet

    @property
    def _data_ready(self):
        self._dbg(f"_data_ready: {self._uart.any()}")
        return self._uart.any() >= 4

    def soft_reset(self):
        """
        UART has its own Soft reset,
        Sends the 0x01 'reset' command over Channel 1 (Executable) 
        to initiate a BNO08X firmware restart.
        
        Section 1.3.1 SHTP states: The executable channel (channel=1) allows the host to reset the BNO08X
        and provide details of its operating mode. use write 1 – reset, read 1 - reset complete.
       """
        self._dbg("*** Soft Reset in UART , using Channel 1 command, starting...")

        # Payload: 0x01 (the 'reset' command)
        reset_payload = bytearray([0x01])
        # Send the packet on Channel 1 (BNO_CHANNEL_EXE = const(1) - 0x01)
        self._send_packet(0x01, reset_payload)

        # flush any uart data leftover from the previous run before the reset
        while self._uart.any():
            self._uart.read(self._uart.any())
        self._dbg("*** Cleared stale UART buffer during reset")
        sleep_ms(500)

        start_time = ticks_ms()
        self._dbg("Process initial packets, until get Product ID report (0xf8)...")

        # Loop for a short period to process reports (like Timestamp or Command Response)
        while _elapsed_sec(start_time) < 1.0:
            try:
                # Check for enough bytes for a header (4) to prevent blocking indefinitely
                if not self._data_ready:
                    sleep_ms(10)
                    continue

                packet = self._read_packet()
                self._handle_packet(packet)
                self._dbg(f"Initial packet, Channel {packet.channel_number} (Seq {packet.sequence_number}).")

            except (RuntimeError, PacketError):
                # expected end-of-burst condition (timeout, no more data)
                break

            except KeyError as e:
                self._dbg(f"exit Soft Teset: minor KeyError caught, likely stream end/data access, Error:{e})")
                break

            except Exception as e:
                self._dbg(f"FATAL UNEXPECTED ERROR during boot processing: (Type: {type(e)}): {e}. Exiting.")
                raise  # Re-raise when unexpected and fatal

        # Reset tx and rx sequence numbers, BNO08X initially sets sequence numbers to 0 after boot.
        self._tx_sequence_number = [0, 0, 0, 0, 0, 0]
        self._rx_sequence_number = [0, 0, 0, 0, 0, 0]

        self._dbg("End Soft RESET in uart.py")

    def hard_reset(self) -> None:
        """
        Hardware reset the sensor to an initial state
        UART handles SHTP protocol and must evaluate command packet response
        """
        if not self._reset_pin:
            return

        self._dbg("*** Hard Reset in UART, starting...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_us(1)  # data sheet 6.5.3 Startup timing says only 10ns needed
        self._reset_pin.value(1)

        # flush any uart data leftover from the previous run before the reset
        while self._uart.any():
            self._uart.read(self._uart.any())
        self._dbg("*** Cleared stale UART buffer during reset")

        sleep_ms(120)  # data sheet 6.5.3 Startup timing implies 94 ms needed

        start_time = ticks_ms()
        self._dbg("Process initial packets, until get Product ID report (0xf8)...")

        # Loop for a short period to process reports (like Timestamp or Command Response)
        while _elapsed_sec(start_time) < 1.0:
            try:
                # Check for enough bytes for a header (4) to prevent blocking indefinitely
                if not self._data_ready:
                    sleep_ms(10)
                    continue

                packet = self._read_packet()
                self._handle_packet(packet)
                self._dbg(f"Initial packet, Channel {packet.channel_number} (Seq {packet.sequence_number}).")

            except (RuntimeError, PacketError):
                # expected end-of-burst condition (timeout, no more data)
                break

            except KeyError as e:
                self._dbg(f"exit hard reset: minor KeyError caught, likely stream end/data access, Error:{e})")
                break

            except Exception as e:
                self._dbg(f"FATAL UNEXPECTED ERROR during boot processing: (Type: {type(e)}): {e}. Exiting.")
                raise  # Re-raise when unexpected and fatal

        # Reset tx and rx sequence numbers, BNO08X initially sets sequence numbers to 0 after boot.
        self._tx_sequence_number = [0, 0, 0, 0, 0, 0]
        self._rx_sequence_number = [0, 0, 0, 0, 0, 0]

        self._dbg("*** Hard Reset acknowledged in uart.py, sequence numbers reset")
        # Control returns to bno08x.py:reset_sensor which calls _check_id()
