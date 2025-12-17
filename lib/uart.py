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

import uctypes
from machine import Pin
from utime import sleep_ms, sleep_us, ticks_ms, ticks_diff

from bno08x import BNO08X, Packet, PacketError

# TODO Need to find definitive value
# 272 bytes shown in ll-test GitHub
# 256 returned by Advertisement debug=True, TAG_MAX_CARGO_PLUS_HEADER_READ
#     BUT, then Arduino code subtracts 4, which is header size?
# 282: x01 x1a   spi header+advert
# 284: x01 x1c   i2c header+advert
_SHTP_MAX_CARGO_PACKET_BYTES = 284

_HEADER_STRUCT = {
    "packet_bytes": (uctypes.UINT16 | 0),
    "channel": (uctypes.UINT8 | 2),
    "sequence": (uctypes.UINT8 | 3),
}


class BNO08X_UART(BNO08X):
    """
    UART-SHTP class for the BNO08x IMUs from CEVA & Hillcrest Laboratories
    """

    def __init__(self, uart, reset_pin=None, int_pin=None, debug=False):
        self._uart = uart
        self._debug = debug
        _interface = "UART"

        if int_pin is None:
            raise RuntimeError("int_pin is required for UART operation")
        if not isinstance(int_pin, Pin):
            raise TypeError("int_pin must be a Pin object, not {type(int_pin)}")
        self._int_pin = int_pin

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"reset_pin (RST) must be a Pin object or None, not {type(reset_pin)}")
        self._reset_pin = reset_pin

        # wake_pin must be NONE!  wake_pin/PS0 = 0 (gnd)
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    def _wait_for_int(self):
        """
        Waits for the BNO08x int_pin to assert (go low) by monitoring 
        the change in the microsecond timestamp set by the ISR.
        """
        initial_int_time = self.last_interrupt_us
        start_time = ticks_ms()

        # Check if the interrupt is already active (was missed)
        if self._int_pin.value() == 0:
            # * comment out self._dbg for normal operation, adds delay even with debug=False
            # self._dbg("int_piun is active low (0) on entry.")
            return 
        
        # Poll the interrupt timestamp for a change
        while ticks_diff(ticks_ms(), start_time) < 10: 
            if self.last_interrupt_us != initial_int_time:
                return 
            sleep_us(100)
        
        raise RuntimeError(f"_wait_for_int timeout ({ticks_diff(ticks_ms(), start_time)}ms) waiting for int_pin")

    def _send_packet(self, channel, data):
        """ 1.2.3.1 UART Operation: Bytes sent to the BNO08X must be separated by at least 100us."""
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4
        byte_buffer = bytearray(1)

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = seq
        self._data_buffer[4: 4 + data_length] = data

        if self._debug:
            packet = Packet(self._data_buffer)
            self._dbg(f"  Sending Packet *************{packet}")

        # ---start--- UART Send packet - handle SHTP protocol
        self._uart.write(b"\x7e")  # start byte
        sleep_us(100)
        self._uart.write(b"\x01")  # SHTP byte
        sleep_us(100)

        # Scan outgoing packet for UART reserved bytes (0x7e or 0x7d), if so escape it
        outgoing_bytes = []
        for b in self._data_buffer[0:write_length]:
            if b == 0x7e or b == 0x7d:
                outgoing_bytes.append(0x7D)  # add UART Control Escape
                outgoing_bytes.append(b ^ 0x20)  # XOR with 0x20
            else:
                outgoing_bytes.append(b)

        # Write all bytes with 100us delay
        byte_buffer = bytearray(1)
        for b in outgoing_bytes:
            byte_buffer[0] = b
            self._uart.write(byte_buffer)
            sleep_us(100)

        # UART end byte
        self._uart.write(b"\x7e")
        sleep_us(100)
        # ----end---- UART Send packet - handle SHTP protocol: 

        self._tx_sequence_number[channel] = (seq + 1) % 256
        return self._tx_sequence_number[channel]

    def _read_into(self, buf, start, end):
        """used to read UART data for all reads """
        for idx in range(start, end):
            data = self._uart.read(1)
            try:
                b = data[0]
            except TypeError as e:
                raise RuntimeError(f"_read_into Timeout reading data byte {idx}") from e

            if b == 0x7D:  # control escape
                data = self._uart.read(1)
                try:
                    b = data[0]
                except TypeError as e:
                    raise RuntimeError(f"_read_into Timeout reading escape sequence byte {idx}") from e
                b ^= 0x20  # control escape

            buf[idx] = b

    def _read_packet(self, wait=None):
        wait = bool(wait)  # both wait=None wait=False are non-blocking
        if wait:
            if self._uart.any() == 0:
                self._wait_for_int() # Will raise a timeout if no new interrupt after 10ms

        # Buffer for Read 4-byte SHTP header and process
        header_mv = memoryview(self._data_buffer)[:4]

        # ---start--- UART Header read - handle SHTP protocol: 
        # UART wait till read 0x7E start byte
        start_time_read = ticks_ms()
        while True:
            data = self._uart.read(1)
            if not data:
                if ticks_diff(ticks_ms(), start_time_read) > 100:
                    return None
                continue
            b = data[0]
            if b == 0x7E:
                break

        # Read the SHTP Protocol ID (0x01)
        data = self._uart.read(1)
        if not data or data[0] != 0x01:
            raise RuntimeError(f"_read_packet header: Didn't find SHTP Protocol ID 0x01, saw {hex(data[0])}")

        # Read header bytes
        self._read_into(header_mv, start=0, end=4)
        # ----end---- UART Header read - handle SHTP protocol: 

        header_view = uctypes.struct(uctypes.addressof(self._data_buffer), _HEADER_STRUCT, uctypes.LITTLE_ENDIAN)
        raw_packet_bytes = header_view.packet_bytes
        channel = header_view.channel
        seq = header_view.sequence
        
        # * comment out self._dbg for normal operation, adds delay even with debug=False
        # self._dbg(f" _read_packet Header {hex(raw_packet_bytes)}, {channel=}, {seq=}")

        self._rx_sequence_number[channel] = seq  # SH2 Sequence number

        if raw_packet_bytes == 0:  # skip 0 lenght
            self._dbg("_read_packet: packet_bytes=0, returning None.")
            return None
        if raw_packet_bytes == 0xFFFF:  # bad sensor data 
            raise OSError(f"FATAL BNO08X Error: Invalid SHTP header(0xFFFF), BNO08x sensor corrupted?")

        packet_bytes = raw_packet_bytes & 0x7FFF

        if packet_bytes > len(self._data_buffer):
            self._data_buffer = bytearray(packet_bytes)

        if packet_bytes <= _SHTP_MAX_CARGO_PACKET_BYTES:
            mv = memoryview(self._data_buffer)[:packet_bytes]

            # ---start--- UART Payload read
            self._read_into(mv, start=4, end=packet_bytes)

            data = self._uart.read(1)
            if not data:
                raise RuntimeError("_read_packet payload: Timeout while waiting for packet end")

            b = data[0]
            if b != 0x7E:
                raise RuntimeError(f"_read_packet payload: Didn't find UART end 0x7e, saw {hex(b)}")
            # ----end---- UART Payload read 

        else:
            print(f"FRAGMENTED PACKET - {packet_bytes=} and {_SHTP_MAX_CARGO_PACKET_BYTES=}")
            print(f"***** NEED to implement multi-packet reads, erasing header")
            print(f"* Have yet to see packet_bytes > 193 bytes, algorithm sketched out")
            print(f"* Ceva and others have no clear documentation of the max cargo bytes value")
            print(f"{self._data_buffer}")
            raise NotImplementedError("The multi-packet reads are NOT unimplemented. TODO")

            # at startup some first packets have continuation, likely missed the packet before
            # when the payload bytes are longer than the xxxxx then we must processess the next packet
            # this should have continuation bit set
            continuation = bool(raw_packet_bytes & 0x8000)
            if continuation:
                self._dbg(f"CONTINUATION in _read_packet: {packet_bytes=}")
                # raise PacketError("read partial packet")

        new_packet = Packet(self._data_buffer[:packet_bytes])
        seq = new_packet.header.sequence_number
        self._rx_sequence_number[channel] = seq  # report sequence number

        # * comment out self._dbg for normal operation, adds 105ms delay even with debug=False
        # self._dbg(f" Received Packet *************{new_packet}")

        return new_packet

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
        sleep_ms(300)  # data sheet 6.5.3 Startup timing implies 94 ms needed

        # flush any uart data leftover from the previous run before the reset
        while self._uart.any():
            self._uart.read(self._uart.any())
        self._dbg("*** UART Cleared stale UART buffer after reset")

        self._dbg("*** Hard Reset End in UART, awaiting acknowledgement (0xf8)")

    def _wake_signal(self):
        """UART has no wake signal, when called in the base class this is a noop"""
        pass

    @property
    def _data_ready(self):
        """UART variant also has uart.any() fallback"""
        #self._dbg(f"_data_ready: {self._uart.any()}")
        return self._uart.any() >= 4
