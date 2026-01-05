# test_reports_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test common sensor reports:
# acceleration, magnetic, gryoscope, quaternion, quaternion.euler
#
# Enabling reports at 4 Hz (~0.25 sec)
# sensor provides frequencies close to what was requested

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms


int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK  
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)

bno.quaternion.enable(100)

# sensor provides frequencies close to what was requested
#bno.print_report_period()

while True:
    # Update required each loop to check if any sensor updated, print sensor data (some or all may be old data)
    # see test_reports_full_spi.py, for example of only printing a sensor when it is updated
    bno.update_sensors()

    if bno.quaternion.updated:
        roll, pitch, yaw = bno.quaternion.euler
        print(f"{roll:+.3f}, {pitch:+.3f}, {yaw:+.3f}")
