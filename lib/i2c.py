# BNO08X Micropython I2C Function by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
I2C Class that requires BNO08X base Class
"""

from struct import pack_into

import uctypes
from machine import Pin
from micropython import const
from utime import ticks_ms, ticks_diff, sleep_us

from bno08x import BNO08X, Packet, PacketError

_BNO08X_DEFAULT_ADDRESS = const(0x4B)
_BNO08X_BACKUP_ADDRESS = const(0x4A)

_HEADER_STRUCT = {
    "packet_bytes": (uctypes.UINT16 | 0),
    "channel": (uctypes.UINT8 | 2),
    "sequence": (uctypes.UINT8 | 3),
}


def _is_i2c(obj) -> bool:
    """ Check that i2c object has required interfaces """
    return (
        hasattr(obj, "readfrom") and
        hasattr(obj, "writeto") and
        hasattr(obj, "readfrom_mem") and
        hasattr(obj, "writeto_mem")
    )


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs on I2C

    Args:
        reset_pin: required to hard reset BNO08x, only reliable way to boot
        int_pin: required int_pin that signals BNO08x
        address: I2C address of sensor, which can often be changed with solder blobs on sensor boards
        debug: prints very detailed logs, primarily for driver debug & development
    """

    def __init__(self, i2c_bus, address=_BNO08X_DEFAULT_ADDRESS, reset_pin=None, int_pin=None, debug=False):
        if not _is_i2c(i2c_bus):
            raise TypeError("i2c_bus must be an I2C object with required interfaces")
        
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
        self._int = int_pin
        self._int.init(Pin.IN, Pin.PULL_UP)  # guarantee int_pin is properly set up

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"Reset (RST) pin must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin
    
        # I2C can not use cs_pin or wake_pin
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    def _wait_for_int(self):
        """
        Waits for the BNO08x H_INTN pin to assert (go low) using the IRQ flag.
        """
        start_time = ticks_ms()

        if self._int.value() == 0 and self._data_ready:
            # self._dbg commented out in time critical code
            # self._dbg("_wait_for_int: INT is active low (0) on entry and _data_ready")
            return

        while ticks_diff(ticks_ms(), start_time) < 3000:  # 3.0sec
            if self._data_ready:
                return
            sleep_us(1000)

        raise RuntimeError("Timeout (3.0s) waiting for INT flag to be set")

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<HBB", self._data_buffer, 0, write_length, channel, seq)
        self._data_buffer[4:4 + data_length] = data

        if self._debug:
            packet = Packet(self._data_buffer)
            self._dbg("")
            self._dbg(f"Sending packet: {packet}")

        mv = memoryview(self._data_buffer)
        self._i2c.writeto(self._bno_i2c_addr, mv[:write_length])

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return self._tx_sequence_number[channel] 
 
    def _read_packet(self, wait=None):
        wait = bool(wait) #  both wait=None wait=False are non-blocking
        if wait:
            self._wait_for_int()

        header_mv = memoryview(self._data_buffer)[:4]
        self._i2c.readfrom_into(self._bno_i2c_addr, header_mv)
        
        header_view = uctypes.struct(uctypes.addressof(self._data_buffer), _HEADER_STRUCT, uctypes.LITTLE_ENDIAN)
        packet_bytes = header_view.packet_bytes
        channel = header_view.channel
        seq = header_view.sequence
        self._rx_sequence_number[channel] = seq # SH2 Sequence number
        
        if packet_bytes > len(self._data_buffer):
            self._data_buffer = bytearray(packet_bytes)
        if packet_bytes == 0:
            return None     

        mv = memoryview(self._data_buffer)[:packet_bytes]
        self._i2c.readfrom_into(self._bno_i2c_addr, mv)

        new_packet = Packet(self._data_buffer[:packet_bytes])
        seq = new_packet.header.sequence_number
        self._rx_sequence_number[channel] = seq # report sequence number
        
        # self._dbg commented out in time critical code
        self._dbg(f"Received Packet: {new_packet}")
        
        return new_packet

    # I2C _data_ready logic. resets _data_available flag for next int event
    @property
    def _data_ready(self):
        if self._int.value() == 0:
            self._data_available = True

        ready = self._data_available
        self._data_available = False
        return ready