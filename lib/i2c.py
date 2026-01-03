# BNO08X Micropython I2C Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
I2C Class that requires BNO08X base Class
"""

from struct import pack

from machine import Pin
from micropython import const
from utime import ticks_us, ticks_diff

from bno08x import BNO08X

_BNO08X_DEFAULT_ADDRESS = const(0x4B)
_BNO08X_BACKUP_ADDRESS = const(0x4A)


def _is_i2c(obj) -> bool:
    """ Check that i2c object has required interfaces """
    return (
            hasattr(obj, "readfrom") and
            hasattr(obj, "writeto") and
            hasattr(obj, "readfrom_mem") and
            hasattr(obj, "writeto_mem")
    )


class BNO08X_I2C(BNO08X):
    """ I2C Interface Library for the BNO08x IMUs

    Args:
        reset_pin: required to hard reset BNO08x, only reliable way to boot
        int_pin: required int_pin that signals BNO08x
        address: I2C address of sensor, which can often be changed with solder blobs on sensor boards
        debug: prints very detailed logs, primarily for driver debug & development
    """

    def __init__(self, i2c_bus, address=_BNO08X_DEFAULT_ADDRESS, reset_pin=None, int_pin=None, debug=False):
        if not _is_i2c(i2c_bus):
            raise TypeError("i2c parameter must be an I2C object")

        self._i2c = i2c_bus
        self._debug = debug
        _interface = "I2C"

        # Validate the i2c address
        if address == _BNO08X_DEFAULT_ADDRESS:
            self._dbg(f"Using default I2C address ({hex(address)})")
        elif address == _BNO08X_BACKUP_ADDRESS:
            self._dbg(f"Using backup I2C address ({hex(address)})")
        else:
            raise ValueError(
                f"Invalid I2C address {hex(address)}, "
                f"Must be {hex(_BNO08X_DEFAULT_ADDRESS)} or {hex(_BNO08X_BACKUP_ADDRESS)}"
            )
        self._bno_i2c_addr = address

        if int_pin is None:
            raise RuntimeError("int_pin is required for I2C operation")
        if not isinstance(int_pin, Pin):
            raise TypeError(f"int_pin must be a Pin object, not {type(int_pin)}.")
        self._int_pin = int_pin
        self._int_pin.init(Pin.IN, Pin.PULL_UP)  # guarantee int_pin is properly set up

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"Reset (RST) pin must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin

        self._header = bytearray(4)  # efficient spi handling of header read
        self._header_mv = memoryview(self._header)

        # I2C can not use cs_pin or wake_pin
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    def _wake_signal(self):
        """I2C has no wake signal, when called in the base class this is a noop"""
        pass

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
        write_length = data_length + 4  # _SHTP_HEADER_LEN=4
        send_packet = bytearray(pack("<HBB", write_length, channel, seq) + data)

        if self._debug:
            self._dbg(f"  Sending Packet *************{self._packet_decode(write_length, channel, seq, data)}")

        self._i2c.writeto(self._bno_i2c_addr, send_packet)

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return self._tx_sequence_number[channel]

    @micropython.native
    def _read_packet(self, wait=False):
        if self._int_pin.value() != 0:
            if not wait or not self._wait_for_int(timeout_us=50000):
                return None

        i2c = self._i2c
        i2c_addr = self._bno_i2c_addr
        h_mv = self._header_mv
        h = self._header

        # Initial 4-byte header
        i2c.readfrom_into(i2c_addr, h_mv)

        raw_packet_bytes = (h[1] << 8) | h[0]
        if raw_packet_bytes == 0 or raw_packet_bytes == 0xFFFF:
            return None

        is_continuation = bool(raw_packet_bytes & 0x8000)
        packet_bytes = raw_packet_bytes & 0x7FFF

        # if fresh packet, clear previous assembly buffer
        if not is_continuation:
            self._assembly_buffer = bytearray()
            self._target_len = packet_bytes

        # self._max_header_plus_cargo set in advertisement (256), originally set to 284 to cover big advertisement packet
        fragment_bytes = min(packet_bytes, self._max_header_plus_cargo)

        if fragment_bytes > len(self._data_buffer):
            self._data_buffer = bytearray(fragment_bytes)

        fragment_mv = memoryview(self._data_buffer)[:fragment_bytes]
        i2c.readfrom_into(i2c_addr, fragment_mv)
        self._assembly_buffer.extend(fragment_mv[4:])  # Append cargo only, skip fragment header

        channel = h[2]
        seq = h[3]
        self._rx_sequence_number[channel] = seq

        if len(self._assembly_buffer) + 4 < self._target_len:
            if self._wait_for_int(timeout_us=10000):
                return self._read_packet(wait=True)  # next header will have continuation bit set

        payload_bytes = len(self._assembly_buffer)
        mv = memoryview(self._assembly_buffer)[:payload_bytes]

        # * comment out self._dbg for normal operation, adds 105ms delay even with debug=False, if self._debug also helps
        # if self._debug:
        #     self._dbg(f" Received Packet *************{self._packet_decode(payload_bytes + 4, channel, seq, mv)}")

        return mv, channel, payload_bytes
