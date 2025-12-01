# test_simple_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test simple sensor report for acceleration

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO sck 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)

print(spi)
print("Start")
print("====================================\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 250)

bno.print_report_period()
print("\nBNO08x sensors enabled")

while True:
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f} m/s²")
    # Notice Gravity acceleration downwards (~9.8 m/s²)
