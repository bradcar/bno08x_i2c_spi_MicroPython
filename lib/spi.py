# BNO08X Micropython SPI Interface by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
SPI Class that requires BNO08X base Class

BNO08x sensors use the non-default SPI settings. This code reconfigures SPI to required settings.
BNO08X Datasheet (1.2.4.2 SPI) requires CPOL = 1 and CPHA = 1, which is: polarity=1 and phase=1

The INT pin is used to tell the host when the BNO08x has data ready to read.
The BNO08x datasheet says the host must respond to H_INTN assertion within ≈10ms to avoid starvation. 

Using SPI, a wait signal is used to signal a  microcontroller-to-BNO write.

Using multiple sensors on SPI - untested with this driver
* Each BNO08x needs its own Chip Select (cs_pin) to each BNO CS pins
* Each BNO08x needs its own Interrupt (int_pin) to each BNO Int pins
* Each BNO08x needs its own Wake (wake_pin) to each BNO Int pins
* they can share the Reset (reset_pin), a reset on one resets all sensors
* they can share the three SPI signals (sck, mosi, miso) new names=(sck, pico, poci)
"""
from struct import pack

from bno08x import BNO08X
from machine import Pin
from utime import ticks_us, ticks_diff, sleep_us


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
        self._wake_pin = wake_pin
        self._wake_pin.value(1)  # wake_pin must be high to select SPI operation

        if cs_pin is None:
            raise RuntimeError("cs_pin is required for SPI operation")
        if not isinstance(cs_pin, Pin):
            raise TypeError("cs_pin must be a Pin object, not {type(cs_pin)}")
        self._cs_pin = cs_pin
        self._cs_pin.value(1)  # ensure CS is de-asserted before communication

        if int_pin is None:
            raise RuntimeError("int_pin is required for SPI operation")
        if not isinstance(int_pin, Pin):
            raise TypeError("int_pin must be a Pin object, not {type(int_pin)}")
        self._int_pin = int_pin
        self._int_pin.init(Pin.IN, Pin.PULL_UP)  # guarantee int_pin is properly set up

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"reset_pin (RST) must be a Pin object or None, not {type(reset_pin)}")
        self._reset_pin = reset_pin

        self._header = bytearray(4)  # SHTP headers are 4 bytes only
        self._header_mv = memoryview(self._header)

        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=cs_pin, wake_pin=wake_pin,
                         debug=debug)

    @micropython.native
    def _wait_for_int(self, timeout_us=1000):
        if self._int_pin.value() == 0:
            return True

        start = ticks_us()
        while self._int_pin.value() != 0:
            if ticks_diff(ticks_us(), start) > timeout_us:
                return False
        return True

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4
        send_packet = bytearray(pack("<HBB", write_length, channel, seq) + data)

        if self._debug:
            self._dbg(f"  Sending Packet *************{self._packet_decode(write_length, channel, seq, data)}")

        self._cs_pin.value(0)
        sleep_us(1)  # BNO08x Figure 6-6: SPI timing, needs > 31ns
        self._spi.write(send_packet)
        self._cs_pin.value(1)

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return

    @micropython.native
    def _read_packet(self, wait=False):
        if self._int_pin.value() != 0:
            if not wait or not self._wait_for_int(timeout_us=50000):
                return None

        spi = self._spi
        cs = self._cs_pin
        h_mv = self._header_mv
        h = self._header

        # Read Header 
        cs.value(0)
        sleep_us(1)
        spi.readinto(h_mv, 0x00)  # CS held low, so only read payload below

        raw_packet_bytes = (h[1] << 8) | h[0]
        if raw_packet_bytes == 0:
            cs.value(1)
            return None  # Must check for None (non-tuple) first, then can unpack data tuple
        if raw_packet_bytes == 0xFFFF:
            cs.value(1)
            raise OSError("FATAL BNO08X Error: Invalid SHTP header(0xFFFF), BNO08x sensor corrupted?")

        packet_bytes = raw_packet_bytes & 0x7FFF
        is_continuation = bool(raw_packet_bytes & 0x8000) # not True for first packet

        # payload fragment to read, advertisement sets _max_header_plus_cargo=256, initial was 284 for big advertisement
        fragment_bytes = min(packet_bytes, self._max_header_plus_cargo) - 4

        # SPI with CS still low, we only read payload
        fragment_mv = memoryview(self._data_buffer)[:fragment_bytes]
        spi.readinto(fragment_mv, 0x00)
        cs.value(1)

        channel = h[2]
        seq = h[3]
        self._rx_sequence_number[channel] = seq

        # Single Packet fast path
        if not is_continuation and packet_bytes <= self._max_header_plus_cargo:
            # * comment out self._dbg for normal operation, self._dbg very slow if uncommented even if debug=False
            # if self._debug:
            #     self._dbg(f" Received Packet *************{self._packet_decode(payload_bytes + 4, channel, seq, mv)}")
            return fragment_mv, channel, fragment_bytes

        # Multipart assembly
        if not is_continuation:
            self._assembly_buffer = bytearray()
            self._target_len = packet_bytes

        self._assembly_buffer.extend(fragment_mv)

        # check and read more fragments
        if len(self._assembly_buffer) + 4 < self._target_len:
            if self._wait_for_int(timeout_us=10000):
                return self._read_packet(wait=True)

        payload_bytes = len(self._assembly_buffer)
        mv = memoryview(self._assembly_buffer)[:payload_bytes]

        # * comment out self._dbg for normal operation, self._dbg very slow if uncommented even if debug=False
        # if self._debug:
        #     self._dbg(f" Received Packet *************{self._packet_decode(payload_bytes + 4, channel, seq, mv)}")

        return mv, channel, payload_bytes
