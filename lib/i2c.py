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
        self._int = int_pin
        self._int.init(Pin.IN, Pin.PULL_UP)  # guarantee int_pin is properly set up

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"Reset (RST) pin must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin

        # I2C can not use cs_pin or wake_pin
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=None, wake_pin=None, debug=debug)

    def _wait_for_int(self):
        """
        Waits for int_pin to assert (go low) by monitoring microsecond timestamp set by the Interrupt.
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
            sleep_us(10)
        
        raise RuntimeError(f"_wait_for_int timeout ({ticks_diff(ticks_ms(), start_time)}ms) waiting for int_pin")

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<HBB", self._data_buffer, 0, write_length, channel, seq)
        self._data_buffer[4:4 + data_length] = data

        if self._debug:
            packet = Packet(self._data_buffer)
            self._dbg(f"  Sending Packet *************{packet}")

        mv = memoryview(self._data_buffer)
        self._i2c.writeto(self._bno_i2c_addr, mv[:write_length])

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return self._tx_sequence_number[channel]

    def _read_packet(self, wait=None):
        wait = bool(wait)  # both wait=None wait=False are non-blocking        
        # In I2C, data is ready if the INT pin is low OR if the user requested to wait.
        if wait or self._int.value() == 0:
            if self._int.value() != 0:
                self._wait_for_int() # Will raise a timeout if no new interrupt after 10ms

        # Read 4-byte SHTP header and process
        header_mv = memoryview(self._data_buffer)[:4]
        self._i2c.readfrom_into(self._bno_i2c_addr, header_mv)

#         header_view = uctypes.struct(uctypes.addressof(self._data_buffer), _HEADER_STRUCT, uctypes.LITTLE_ENDIAN)
#         raw_packet_bytes = header_view.packet_bytes
#         channel = header_view.channel
#         seq = header_view.sequence
# TODO remove uctypes
        
        raw = bytes(header_mv)  # forces materialization of bytearray
        raw_packet_bytes = raw[0] | (raw[1] << 8)
        channel = raw[2]
        seq = raw[3]

        self._rx_sequence_number[channel] = seq  # SH2 Sequence number

        if raw_packet_bytes == 0:  # Fast return, if only SHTP header
            return None
        if raw_packet_bytes == 0xFFFF:  # bad sensor         
            raise OSError(f"FATAL BNO08X Error: Invalid SHTP header(0xFFFF), BNO08x sensor corrupted?")
        
        packet_bytes = raw_packet_bytes & 0x7FFF

        if packet_bytes > len(self._data_buffer):
            self._data_buffer = bytearray(packet_bytes)

        if packet_bytes <= _SHTP_MAX_CARGO_PACKET_BYTES:
            mv = memoryview(self._data_buffer)[:packet_bytes]
            self._i2c.readfrom_into(self._bno_i2c_addr, mv)
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

    def _wake_signal(self):
        """I2C has no wake signal, when called in the base class this is a noop"""
        pass