# BNO08X Micropython SPI Interface by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
SPI Class that requires BNO08X base Class

BNO08x sensor use the non-defaul SPI. This driver reconfigures SPI to those settings.
BNO08X Datasheet (1.2.4.2 SPI) requires CPOL = 1 and CPHA = 1, which is: polarity=1 and phase=1

The BNO08x's SPI protocol has two main transactions:
1) Host → BNO08x (Write Command): The host initiates the transfer to send a command or data.
2) Host ← BNO08x (Read Data): The host initiates the transfer to read the BNO08x's data.

The INT pin is used to tell the host when the BNO08x has data ready (for a Read).
Requiring an active-low INT signal before the host sends a command (a Write) is overly strict.
The BNO08x documentation indicates that for a host-to-BNO write, the host is usually free to
initiate the transfer.

TODO: The BNO08x datasheet says the host must respond to H_INTN assertion within ≈10ms
to avoid starvation. While the 3.0s timeout prevents lockup, the sleep_ms(10) in
the loop means the driver will frequently miss the 10ms deadline when polling.

Ideas for multiple sensors on SPI - untested & this driver may need more code
Each Sensor needs:
* its own Chip Select (cs_pin) to each BNO CS pins
* its own Interrupt (int_pin) to each BNO Int pins
* they can share the Reset (reset_pin) which must be connected to all the BNO RST pins.
* they can share the three SPI signals which must be connected to all the BNOs.

"""
from struct import pack_into

from machine import Pin
from utime import ticks_ms, ticks_diff, sleep_us

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE

# TODO Need to find definitive value for max_cargo
# 272 bytes shown in ll-test GitHub
# 256 returned by Advertisement debug=True, TAG_MAX_CARGO_PLUS_HEADER_READ
#     BUT, then Arduino code subtracts 4, which is header size?
# 282: x01 x1a   spi header+advert
# 284: x01 x1c   i2c header+advert
_SHTP_MAX_CARGO_PACKET_BYTES = 284


def _is_spi(obj) -> bool:
    """Check that SPI object has required interfaces"""
    return (hasattr(obj, "read") and
            hasattr(obj, "write") and
            hasattr(obj, "write_readinto") and
            hasattr(obj, "init")
            )


class BNO08X_SPI(BNO08X):
    """Library for the BNO08x IMUs on SPI

    Args:
        spi_bus: SPI bus object
        cs_pin: SPI CS pin to signal reads or writes
        reset_pin: optionl reset to BNO08x
        int_pin=None: optional int_pin to get signal when BNO08x is ready
        baudrate: (default 1 MHz, max 3 MHz)
        debug: prints very detailed logs, primarily for driver debug & development.
    """

    def __init__(self, spi_bus, cs_pin, reset_pin=None, int_pin=None, wake_pin=None, debug=False):
        if not _is_spi(spi_bus):
            raise TypeError("spi parameter must be an SPI object")

        # BNO08X Datasheet (1.2.4.2 SPI) requires CPOL = 1 and CPHA = 1, which is: polarity=1 and phase=1
        self._spi = spi_bus
        self._spi.init(polarity=1, phase=1)
        self._debug = debug
        _interface = "SPI"

        if wake_pin is None:
            raise RuntimeError("wake_pin is required for SPI operation")
        if not isinstance(wake_pin, Pin):
            raise TypeError("wake_pin must be a Pin object, not {type(wake_pin)}")
        self._wake = wake_pin
        self._wake.value(1)  # wake_pin must be high to select SPI operation

        if cs_pin is None:
            raise RuntimeError("cs_pin is required for SPI operation")
        if not isinstance(cs_pin, Pin):
            raise TypeError("cs_pin must be a Pin object, not {type(cs_pin)}")
        self._cs = cs_pin
        self._cs.value(1)  # ensure CS is de-asserted before communication

        if int_pin is None:
            raise RuntimeError("int_pin is required for SPI operation")
        if not isinstance(int_pin, Pin):
            raise TypeError("int_pin must be a Pin object, not {type(int_pin)}")
        self._int = int_pin
        self._int.init(Pin.IN, Pin.PULL_UP)  # guarantee int_pin is properly set up

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"reset_pin (RST) must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin

        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=cs_pin, wake_pin=wake_pin,
                         debug=debug)

    def _wait_for_int(self):
        """
        Waits for the BNO08x H_INTN pin to assert (go low) using the IRQ flag.
        This resolves the 10ms starvation issue caused by polling.
        """
        initial_int_time = self.last_interrupt_us
        start_time = ticks_ms()

        self._wake_signal()

        if self._int.value() == 0:
            # self._dbg("INT is active low (0) on entry.")
            return

        # Poll the interrupt timestamp for a change
        while ticks_diff(ticks_ms(), start_time) < 10:
            if self.last_interrupt_us != initial_int_time:
                return
                # sleep_us(100)

        raise RuntimeError(f"_wait_for_int timeout ({ticks_diff(ticks_ms(), start_time)}ms) waiting for int_pin")

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4
        pack_into("<HBB", self._data_buffer, 0, write_length, channel, seq)

        mv = memoryview(self._data_buffer)
        mv[4:4 + data_length] = data

        if self._debug:
            packet = Packet(self._data_buffer)
            self._dbg(f"  Sending Packet *************{packet}")

        self._cs.value(0)
        sleep_us(1)
        self._spi.write(mv[:write_length])  # also zero-copy
        self._cs.value(1)

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return

    def _read_header(self, wait=True):
        """Reads the first 4 bytes available as a header"""
        if wait:
            self._wait_for_int()
        else:
            # only attempt the SPI read if INT is LOW.
            if self._int.value() != 0:
                raise PacketError("INT pin high, aborting read: No data ready.")

        self._cs.value(0)
        sleep_us(1)
        mv = memoryview(self._data_buffer)
        self._spi.readinto(mv[:4], 0x00)
        self._cs.value(1)

        # * commented out self._dbg in time critical loops for normal operation
        #  self._dbg(f"_read_packet header: {[hex(x) for x in self._data_buffer[0:4]]}")

    def _read_packet(self, wait=True):
        self._read_header(wait=wait)
        sleep_us(100)

        raw = bytes(self._data_buffer)  # forces materialization of bytearray
        raw_packet_bytes = raw[0] | (raw[1] << 8)
        channel = raw[2]
        seq = raw[3]

        self._rx_sequence_number[channel] = seq  # SH2 Sequence number

        if raw_packet_bytes == 0:  # Fast return, if only SHTP header
            return None
        if raw_packet_bytes == 0xFFFF:  # bad sensor 
            raise PacketError(f"Invalid SHTP header length detected: {hex(raw_packet_bytes)}")

        packet_bytes = raw_packet_bytes & 0x7FFF

        if packet_bytes > DATA_BUFFER_SIZE:  # if packet too big, reallocate, this is normal.
            self._data_buffer = bytearray(packet_bytes)

        self._cs.value(0)
        sleep_us(1)
        mv = memoryview(self._data_buffer)[0:packet_bytes]
        self._spi.readinto(mv, 0x00)
        self._cs.value(1)

        #         continuation = bool(raw_packet_bytes & 0x8000)
        #         if continuation:
        #             self._dbg(f"CONTINUATION in _read_packet: {packet_bytes=}")
        #             # raise PacketError("read partial packet")

        new_packet = Packet(bytes(self._data_buffer[:packet_bytes]))
        self._rx_sequence_number[channel] = new_packet.header.sequence_number  # report sequence number

        # * commented out self._dbg in time critical loops for normal operation, add ??ms even with debug=False
        # self._dbg(f" Received Packet *************{new_packet}")

        return new_packet
