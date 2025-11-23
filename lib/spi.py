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

"""
from struct import pack_into

import uctypes
from machine import Pin
from utime import ticks_ms, ticks_diff, sleep_ms, sleep_us

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE


_HEADER_STRUCT = {
    "packet_bytes": (uctypes.UINT16 | 0),
    "channel": (uctypes.UINT8 | 2),
    "sequence": (uctypes.UINT8 | 3),
}


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

    def __init__(self, spi_bus, cs_pin, reset_pin=None, int_pin=None, wake_pin=None, baudrate=1_000_000, debug=False):

        # BNO08X Datasheet (1.2.4.2 SPI) requires CPOL = 1 and CPHA = 1, which is: polarity=1 and phase=1
        self._spi = spi_bus
        self._spi.init(baudrate=baudrate, polarity=1, phase=1)
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
        self._int.init(Pin.IN, Pin.PULL_UP) # guarantee int_pin is properly set up

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"reset_pin (RST) must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin

        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=cs_pin, wake_pin=wake_pin,
                         debug=debug)

    def _wake_signal(self):
        # UART operation reqires wake_pin is None
        if self._wake_pin is not None:
            self._dbg("WAKE Pulse for BNO08x (spi.py)")
            self._wake_pin.value(0)
            sleep_us(500)  # 500us seems reliable
            self._wake_pin.value(1)
            sleep_ms(1)  # 1 ms works

    def _wait_for_int(self):
        """
        Waits for the BNO08x H_INTN pin to assert (go low) using the IRQ flag.
        This resolves the 10ms starvation issue caused by polling.
        """
        start_time = ticks_ms()

        if self._wake is not None and self._wake.value() == 1:
            self._wake_signal()

        if self._int.value() == 0:
            self._data_available = True  # Ensure the flag is set if we missed the interrupt
            self._dbg("INT is active low (0) on entry.")
            return

        while ticks_diff(ticks_ms(), start_time) < 3000:  # 3.0sec
            if self._data_available:
                self._data_available = False  # Clear the flag for the next event
                return
            sleep_us(10)  # 10 us give 5.4ms loop 1ms give 5.4ms loop
        raise RuntimeError("Timeout (3.0s) waiting for INT flag to be set")

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
        mv = memoryview(self._data_buffer)[:4]
        self._spi.readinto(mv, 0x00)
        self._cs.value(1)

        header_view = uctypes.struct(uctypes.addressof(mv), _HEADER_STRUCT, uctypes.LITTLE_ENDIAN)

        if not wait:
            packet_length = header_view.packet_bytes

            if packet_length == 0xFFFF or packet_length == 0:
                raise PacketError("No valid packet received despite INT being low")

            # self._dbg(f"Non-blocking read SUCCESS. Header length: {packet_length}. Pin state: {self._int.value()}")

    def _read_packet(self, wait=True):
        try:
            self._read_header(wait=wait)
        except PacketError:
            # if wait=False and no packet was available (length 0 or 0xFFFF).
            raise
        except RuntimeError as e:
            if not wait and "Timeout" in str(e):
                raise PacketError("No packet available")
            raise

        halfpacket = False
        mv = memoryview(self._data_buffer)
        header_view = uctypes.struct(uctypes.addressof(mv[:4]), _HEADER_STRUCT, uctypes.LITTLE_ENDIAN)
        packet_bytes = header_view.packet_bytes
        if packet_bytes & 0x8000:
            halfpacket = True
        channel = header_view.channel
        sequence = header_view.sequence

        self._rx_sequence_number[channel] = sequence
        # self._dbg(f"channel {channel} has {packet_bytes - 4} bytes available")

        if packet_bytes > DATA_BUFFER_SIZE:
            # If the packet is too big, reallocate the buffer. This is normal.
            self._data_buffer = bytearray(packet_bytes)

        self._cs.value(0)
        sleep_us(1)
        mv = memoryview(self._data_buffer)[0:packet_bytes]
        self._spi.readinto(mv, 0x00)
        self._cs.value(1)

        if halfpacket:
            raise PacketError("read partial packet")

        new_packet = Packet(self._data_buffer)
        self._update_sequence_number(new_packet)
        return new_packet

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4
        pack_into("<HBB", self._data_buffer, 0, write_length, channel, seq)

        mv = memoryview(self._data_buffer)
        mv[4:4 + data_length] = data

        self._cs.value(0)
        sleep_us(1)
        self._spi.write(mv[:write_length])  # also zero-copy
        self._cs.value(1)

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return
