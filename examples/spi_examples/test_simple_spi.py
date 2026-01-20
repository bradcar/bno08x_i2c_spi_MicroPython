# test_simple_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test simple sensor report for acceleration
#
# Reports requested at 100 Hz, sensor chooses 125Hz (8 msec between reports)
# It prints so much data that your console may not keep up.

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)
print(spi)

print("Start")
print("====================================\n")

bno.acceleration.enable(100)
bno.print_report_period()

while True:
    # Update required to refresh sensor data
    bno.update_sensors()

    if bno.acceleration.updated:
        accel_x, accel_y, accel_z = bno.acceleration
        print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}")

        # Notice Gravity acceleration downwards (about +9.8 m/s²)
