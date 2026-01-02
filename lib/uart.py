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

from struct import pack

import micropython
import uctypes
from machine import Pin
from utime import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff

from bno08x import BNO08X

# TODO Need to find definitive value
# 272 bytes shown in ll-test GitHub
# 256 returned by Advertisement debug=True, TAG_MAX_CARGO_PLUS_HEADER_READ
#     BUT, then Arduino code subtracts 4, which is header size?
# 252: Advertisement spi, i2c, and uart:  header+payload = 256
_SHTP_MAX_CARGO_PACKET_BYTES = 284

class BNO08X_UART(BNO08X):
    """
    UART-SHTP class for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, uart, reset_pin=None, int_pin=None, debug=False):
        self._uart = uart
        self._debug = debug
        _interface = "UART"
        while self._uart.any():
            self._uart.read(self._uart.any())

        if int_pin is None:
            raise RuntimeError("int_pin is required for UART operation")
        if not isinstance(int_pin, Pin):
            raise TypeError("int_pin must be a Pin object, not {type(int_pin)}")
        self._int_pin = int_pin

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"reset_pin (RST) must be a Pin object or None, not {type(reset_pin)}")
        self._reset_pin = reset_pin
        
        self._header = bytearray(4)  # efficient spi handling of header read
        self._header_mv = memoryview(self._header)
        self._byte_buf = bytearray(1)  # efficient spi handling of header read

        # wake_pin must be NONE!  wake_pin/PS0 = 0 (gnd)
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    @micropython.native
    def _wait_for_int(self, timeout_us=1000):
        if self._int_pin.value() == 0:
            return

        start = ticks_us()
        # Check the raw pin value directly
        while True:
            if self._int_pin.value() == 0:
                return

            if ticks_diff(ticks_us(), start) > timeout_us:
                break

        raise RuntimeError("BNO08X UART _wait_for_int Timeout: 1ms exceeded")

    def _soft_reset(self):
        """
        UART has its own Soft reset,
        Sends the 0x01 'reset' command over Channel 1 (Executable) 
        to initiate a BNO08X firmware restart.
        
        Section 1.3.1 SHTP states: The executable channel (channel=1) allows the host to reset the BNO08X
        and provide details of its operating mode. use write 1 – reset, read 1 - reset complete.
       """
        self._dbg("*** Soft Reset in UART , using Channel 1 command, starting...")

        # Reset Command: Payload: 0x01 ('reset' command), sent on BNO_CHANNEL_EXE1 (1)
        reset_payload = bytearray([0x01])
        self._send_packet(0x01, reset_payload)

        # flush any uart data leftover from the previous run before the reset
        while self._uart.any():
            self._uart.read(self._uart.any())
        self._dbg("*** Cleared stale UART buffer during reset")
        sleep_ms(500)

        self._dbg("End Soft RESET in uart.py")

    def _hard_reset(self) -> None:
        """
        Hardware reset the sensor to an initial state
        UART handles SHTP protocol and must evaluate command packet response
        """
        if not self._reset_pin:
            return

        while self._uart.any():
            self._uart.read(self._uart.any())
        self._dbg("*** UART Cleared stale UART buffer before reset")

        self._dbg("*** UART Hard Reset starting...")
        self._reset_pin.value(1)
        sleep_ms(10)
        self._reset_pin.value(0)
        sleep_us(10)  # sleep_us(1), data sheet say only 10ns required,
        self._reset_pin.value(1)
        sleep_ms(100)  # data sheet 6.5.3 Startup timing implies 94 ms needed

        self._dbg("*** Hard Reset End in UART, awaiting acknowledgement (0xf8)\n")

    def _wake_signal(self):
        """UART has no wake signal, when called in the base class this is a noop"""
        pass

    def _send_packet(self, channel, data):
        """ 1.2.3.1 UART Operation:"Bytes sent to the BNO08X must be separated by at least 100us."""
        seq = self._tx_sequence_number[channel]
        self._dbg(f"DEBUG: Sending on Chan {channel} with TX Seq {seq}")
        data_length = len(data)
        write_length = data_length + 4
        send_packet = bytearray(pack("<HBB", write_length, channel, seq) + data)

        if self._debug:
            self._dbg(f"  Sending Packet *************{self._packet_decode(write_length, channel, seq, data)}")

        # ---start--- UART Send packet - handle SHTP protocol
        self._uart.write(b"\x7e")  # start byte
        sleep_us(100)
        self._uart.write(b"\x01")  # SHTP byte
        sleep_us(100)

        # Escape reserved bytes (0x7E or 0x7D)
        for b in send_packet:
            if b in (0x7E, 0x7D):
                self._uart.write(bytes([0x7D, b ^ 0x20]))
            else:
                self._uart.write(bytes([b]))
            sleep_us(100)

        # UART end byte
        self._uart.write(b"\x7e")
        sleep_us(100)
        # ----end---- UART Send packet - handle SHTP protocol: 

        self._tx_sequence_number[channel] = (seq + 1) % 256
        return self._tx_sequence_number[channel]

    def _read_into(self, buf, start, end):
        """Used to read UART data for all reads. Handles 0x7D escape sequences."""
        
        # Local references to avoid repeated attribute lookups
        uart = self._uart
        byte_buf = self._byte_buf
        
        idx = start
        while idx < end:
            # Read exactly 1 byte into our pre-allocated buffer
            if uart.readinto(byte_buf, 1) == 0:
                raise RuntimeError(f"_read_into Timeout: No data at byte {idx}")

            b = byte_buf[0]

            if b == 0x7D:  # Control escape detected
                # Read the next byte to be transformed
                if uart.readinto(byte_buf, 1) == 0:
                    raise RuntimeError(f"_read_into Timeout: Missing byte after escape at {idx}")
                
                b = byte_buf[0] ^ 0x20  # Transform the byte

            buf[idx] = b
            idx += 1

    def _read_packet(self, wait=None):
        if not self._new_data_interrupt and self._uart.any() < 1:
            return None
        
        # Local references to avoid repeated attribute lookups
        uart = self._uart
        byte_buf = self._byte_buf
        h = self._header

        # ---start--- UART Header read - handle SHTP protocol: 
        # UART read until read 0x7E start byte
        start_time_read = ticks_ms()
        while True:
            # Replaced self._uart.read(1) with readinto
            if uart.readinto(byte_buf, 1) == 0:
                if ticks_diff(ticks_ms(), start_time_read) > 100:
                    return None
                continue
            if byte_buf[0] == 0x7E:
                break

        # Skip any additional 0x7E bytes
        while True:
            if uart.readinto(byte_buf, 1) == 0:
                return None
            if byte_buf[0] != 0x7E:
                break

        # Check the SHTP Protocol ID (0x01)
        # self._byte_buf[0] currently holds the first byte after the 0x7E sequence
        if byte_buf[0] != 0x01:
            return None

        # Read header bytes (uses the refactored _read_into internally)
        self._read_into(self._header_mv, start=0, end=4)

        raw_packet_bytes = h[0] | (h[1] << 8)
        if raw_packet_bytes == 0:
            return None 
            
        # if raw_packet_bytes == 0xFFFF: 
        #     raise OSError(f"FATAL BNO08X Error: Invalid SHTP header(0xFFFF), BNO08x sensor corrupted?")

        payload_bytes = (raw_packet_bytes & 0x7FFF) - 4

        if payload_bytes > len(self._data_buffer):
            self._data_buffer = bytearray(payload_bytes)

        if payload_bytes <= _SHTP_MAX_CARGO_PACKET_BYTES:
            mv = memoryview(self._data_buffer)[:payload_bytes]
            # Read payload using refactored _read_into
            self._read_into(mv, start=0, end=payload_bytes)

            # Replaced self._uart.read(1) with readinto for packet termination
            if uart.readinto(byte_buf, 1) == 0:
                raise RuntimeError("_read_packet payload: Timeout while waiting for packet end")

            if byte_buf[0] != 0x7E:
                return None

        else:
            print(f"FRAGMENTED PACKET - {raw_packet_bytes=} and {_SHTP_MAX_CARGO_PACKET_BYTES=}")
            print(f"***** NEED to implement multi-packet reads, erasing header")
            print(f"* Have yet to see raw_packet_bytes > 193 bytes, algorithm sketched out")
            print(f"* Ceva and others have no clear documentation of the max cargo bytes value")
            print(f"{self._data_buffer}")
            raise NotImplementedError("The multi-packet reads are NOT unimplemented. TODO")

            # at startup some first packets have continuation, likely missed the packet before
            # when the payload bytes are longer than the xxxxx then we must processess the next packet
            # this should have continuation bit set
            continuation = bool(raw_packet_bytes & 0x8000)
            if continuation:
                self._dbg(f"CONTINUATION in _read_packet: {raw_packet_bytes=}")
                # raise PacketError("read partial packet")

        channel = h[2]
        seq = h[3]
        self._rx_sequence_number[channel] = seq  # report sequence number

        # * comment out self._dbg for normal operation, adds 105ms delay even with debug=False, if self._debug also helps
        # if self._debug: self._dbg(f" Received Packet *************{self._packet_decode(payload_bytes + 4, channel, seq, mv)}")

        return  mv, channel, payload_bytes
