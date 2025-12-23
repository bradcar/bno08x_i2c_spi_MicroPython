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
from struct import pack_into, pack

import uctypes
from machine import Pin
from utime import ticks_ms, ticks_us, ticks_diff, sleep_us, sleep_ms

from bno08x import BNO08X, Packet, DATA_BUFFER_SIZE

# TODO Need to find definitive value
# 272 bytes shown in ll-test GitHub
# 252 bytes Advertisement return payload length, TAG_MAX_CARGO_PLUS_HEADER_READ
#     BUT, then Arduino code subtracts 4, which is header size?
# 252: uart payload max in  Advertisement
# ???: i2c payload max in  Advertisement

_SHTP_MAX_CARGO_PACKET_BYTES = 284

_HEADER_STRUCT = {
    "packet_bytes": (uctypes.UINT16 | 0),
    "channel": (uctypes.UINT8 | 2),
    "sequence": (uctypes.UINT8 | 3),
}


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
        
        self._header = bytearray(4)  #efficient spi handling of header read

        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=cs_pin, wake_pin=wake_pin,
                         debug=debug)

#     def _wait_for_int(self):
#         """
#         Waits for the BNO08x H_INTN pin to assert (go low) using the IRQ flag.
#         This resolves the 10ms starvation issue caused by polling.
#         """
#         initial_int_time = self.last_interrupt_us
#         start_time = ticks_ms()
#   
#         self._wake_signal()
# 
#         if self._int_pin.value() == 0:
#             # self._dbg("INT is active low (0) on entry.")
#             return
# 
#         # Poll the interrupt timestamp for a change
#         while ticks_diff(ticks_ms(), start_time) < 10: 
#             if self.last_interrupt_us != initial_int_time:
#                 return 
#             # sleep_us(100)
#         
#         raise RuntimeError(f"_wait_for_int timeout ({ticks_diff(ticks_ms(), start_time)}ms) waiting for int_pin")

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
    
    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4
        send_packet = bytearray(pack("<HBB", write_length, channel, seq) + data)

        if self._debug:
            packet = Packet(send_packet)
            self._dbg(f"  Sending Packet *************{packet}")

        self._cs_pin.value(0)
        sleep_us(1)
        self._spi.write(send_packet)
        self._cs_pin.value(1)

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        sleep_ms(10)
        return

    def _read_packet(self, wait=None):
        wait = bool(wait)  # both wait=None wait=False are non-blocking
                
        if wait and self._int_pin.value() != 0:
            self._wait_for_int()

        header_mv = memoryview(self._header)
        # ---start--- SPI Header read
        self._cs_pin.value(0)
        sleep_us(1)
        self._spi.readinto(header_mv, 0x00)
        # Not:e CS is still 0, must raise to (1) after payload read, or when errors out
        # ----end---- SPI Header read 

        header_view = uctypes.struct(uctypes.addressof(self._header), _HEADER_STRUCT, uctypes.LITTLE_ENDIAN)
        raw_packet_bytes = header_view.packet_bytes
        if raw_packet_bytes == 0:  # fast return if 0 payload
            self._cs_pin.value(1)
            #self._dbg("_read_packet: packet_bytes=0, returning None.")
            return None
        
        channel = header_view.channel
        seq = header_view.sequence
        # * comment out self._dbg for normal operation, adds delay even with debug=False
        # self._dbg(f"packet header: {packet_bytes=} {channel=} {seq=} ")

        self._rx_sequence_number[channel] = seq  # SH2 Sequence number

        if raw_packet_bytes == 0xFFFF:  # this shows bad sensor
            self._cs_pin.value(1)
            raise OSError(f"FATAL BNO08X Error: Invalid SHTP header(0xFFFF), BNO08x sensor corrupted?")

        packet_bytes = raw_packet_bytes & 0x7FFF

        if packet_bytes > len(self._data_buffer):
            self._data_buffer = bytearray(packet_bytes)

        if packet_bytes <= _SHTP_MAX_CARGO_PACKET_BYTES:
            
            # read the payload, note since CS never de-asserted this neverre-reads the header
            self._data_buffer[0:4] = self._header
            mv = memoryview(self._data_buffer)[4:packet_bytes]
            
            # ---start--- SPI Payload read
            self._spi.readinto(mv, 0x00)
            self._cs_pin.value(1)  
            # ----end---- SPI Payload read

        else:
            print(f"FRAGMENTED PACKET spi.py: {hex(raw_packet_bytes)=} {packet_bytes=} > {_SHTP_MAX_CARGO_PACKET_BYTES}(max cargo)")
            print(f"FRAGMENTED PACKET spi.py: {self._header=}")
            print(f"***** NEED to implement multi-packet reads, erasing header")
            print(f"* Have yet to see packet_bytes > 193 bytes, algorithm sketched out")
            print(f"* Ceva and others have no clear documentation of the max cargo bytes value")
            print(f" _read_packet Header {hex(raw_packet_bytes)=}, {channel=}, {seq=}")
            print(f"{self._data_buffer[0:16]}")
            raise NotImplementedError("Continuation Packets after Packets are NOT implemented. TODO")

            # at startup some first packets have continuation, likely missed the packet before
            # when the payload bytes are longer than the xxxxx then we must processess the next packet
            # this should have continuation bit set
            continuation = bool(raw_packet_bytes & 0x8000)
            if continuation:
                self._dbg(f"CONTINUATION in _read_packet: {packet_bytes=}")
                # raise PacketError("read partial packet")

        new_packet = Packet(bytes(self._data_buffer[:packet_bytes]))
        #new_packet = Packet(self._data_buffer[:packet_bytes])


        seq = new_packet.header.sequence_number
        self._rx_sequence_number[channel] = seq  # report sequence number

        # * comment out self._dbg for normal operation, adds 105ms delay even with debug=False
        # self._dbg(f" Received Packet *************{new_packet} {packet_bytes=}")
        
        # TODO REMOVE 10 MSEC SLEEP after fix _check_id, current _check_id may call before interrupt hit
        # sleep_ms(10)

        return new_packet
