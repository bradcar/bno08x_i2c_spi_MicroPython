# BNO08X Micropython SPI Interface by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use SPI

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

from utime import ticks_ms, sleep_ms, sleep_us
from machine import Pin

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE, _elapsed_sec


class BNO08X_SPI(BNO08X):
    """Library for the BNO08x IMUs on SPI

    Args:
        spi_bus: SPI bus object
        cs_pin: SPI CS pin to signal reads or writes
        reset_pin: optionl reset to BNO08x
        int_pin=None: optional int_pin to get signal when BNO08x is ready
        baudrate: (default 1 MHz, max 3 MHz)
        debug: Enables optional logging prints used for debugging driver
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
        self._wake.value(1) # wake_pin must be high to select SPI operation

        if cs_pin is None:
            raise RuntimeError("cs_pin is required for SPI operation")
        if not isinstance(cs_pin, Pin):
            raise TypeError("cs_pin must be a Pin object, not {type(cs_pin)}")
        self._cs = cs_pin
        self._cs.value(1) # ensure CS is de-asserted before communication
        
        if int_pin is None:
            raise RuntimeError("int_pin is required for SPI operation")
        if not isinstance(int_pin, Pin):
            raise TypeError("int_pin must be a Pin object, not {type(int_pin)}")
        self._int = int_pin

        if reset_pin is not None and not isinstance(reset_pin, Pin):
            raise TypeError(f"reset_pin (RST) must be a Pin object or None, not {type(reset_pin)}")
        self._reset = reset_pin
        
        super().__init__(_interface, reset_pin=reset_pin, int_pin=int_pin, cs_pin=cs_pin, wake_pin=wake_pin, debug=debug)


    def _wake_signal(self):
        # UART operation reqires wake_pin is None
        if self._wake_pin is not None:
            self._dbg("WAKE Pulse for BNO08x (spi.py)")
            self._wake_pin.value(0)
            sleep_ms(2)  # lower than 1ms doesn't seem to work
            self._wake_pin.value(1)
            sleep_ms(10)  # 1 ms works, 1 ms sometimes fails

    def _wait_for_int(self):
        """
        1.2.4.1: On the BNO085/BNO086, if the host fails to respond to the assertion of H_INTN within approximately 10 ms, the
        BNO085/BNO086 will time out, de-assert H_INTN and retry the operation. Delays in responding to H_INTN cause
        lost processing time on the BNO085/BNO086. Frequent delays will cause process starvation and some
        calculations will not be completed. The result is that some outputs will have errors. To avoid this problem H_INTN
        should be typically handled within 1/10 of the fastest sensor period.
        Fastest sensor period is acceleration at 4ms

        """
        start_time = ticks_ms()

        # SPI operation Requires wake_pin
        # I2C & UARToperation reqires NO wake_pin (None)
        if self._wake is not None and self._wake.value() == 1:
            self._wake_signal()

        if self._int.value() == 0:
            self._dbg("INT is active low (0) on entry.")
            return

        self._dbg("Waiting for INT to go low (active)...")
        while _elapsed_sec(start_time) < 3.0:
            if self._int.value() == 0:
                self._dbg(f"INT went active low after {_elapsed_sec(start_time):.3f}s")
                return

            if self._debug and ticks_ms() % 500 < 5:  # Log every ~0.5 seconds
                self._dbg(f"INT value still high (1) at T={_elapsed_sec(start_time):.3f}s")

            sleep_us(100) # TODO How long?

        self._dbg(f"Timeout (3.0s) reached. INT pin state: {self._int.value()}")
        raise RuntimeError("Timeout waiting for INT to go low")


    def _read_into(self, buf, start=0, end=None):

        if end is None:
            end = len(buf)
        if end <= start:
            return

        self._cs.value(0)
        sleep_us(1)
        mv = memoryview(buf)[start:end]
        self._spi.readinto(mv, 0x00)
        self._cs.value(1)

    def _read_header(self, wait=True):
        """Reads the first 4 bytes available as a header"""

        if wait:
            self._wait_for_int()
        else:
            # only attempt the SPI read if INT is LOW.
            if self._int.value() != 0:
                self._dbg("Not waiting for INT. Pin is HIGH (1). Aborting read: No data ready.")
                raise PacketError("INT pin high, no data.")
            self._dbg("Not waiting for INT. Pin is LOW (0). Proceeding to read...")

        self._cs.value(0)
        sleep_us(1)
        mv = memoryview(self._data_buffer)[:4]
        self._spi.readinto(mv, 0x00)
        self._cs.value(1)

        if not wait:
            packet_len = self._data_buffer[0] | (self._data_buffer[1] << 8)

            if packet_len == 0xFFFF or packet_len == 0:
                self._dbg(f"Non-blocking read FAILED despite INT being low. Header length: {hex(packet_len)}")
                raise PacketError("No valid packet received despite INT being low")

            self._dbg(f"Non-blocking read SUCCESS. Header length: {packet_len}. Pin state: {self._int.value()}")

        self._dbg("")

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

        if self._data_buffer[1] & 0x80:
            halfpacket = True

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._rx_sequence_number[channel_number] = sequence_number

        # Redundant check if the non-blocking logic worked, kept for robustness
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel_number, packet_byte_count - 4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            # If the packet is too big, reallocate the buffer. This is normal.
            self._data_buffer = bytearray(packet_byte_count)

        self._read_into(self._data_buffer, start=0, end=packet_byte_count)
        # print("Packet: ", [hex(i) for i in self._data_buffer[0:packet_byte_count]])

        if halfpacket:
            raise PacketError("read partial packet")

        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)
        self._update_sequence_number(new_packet)
        return new_packet

    def _send_packet(self, channel, data):
        seq = self._tx_sequence_number[channel]
        data_length = len(data)
        write_length = data_length + 4
        pack_into("<HBB", self._data_buffer, 0, write_length, channel, seq)

        mv = memoryview(self._data_buffer)
        mv[4:4+data_length] = data

        self._cs.value(0)
        sleep_us(1)
        self._spi.write(mv[:write_length])  # also zero-copy
        self._cs.value(1)

        self._tx_sequence_number[channel] = (seq + 1) & 0xFF
        return seq


