# test_simple.py
#
# BNO08x MicroPython I2C Test
#
# I2C interface: Test simple sensor report for acceleration

from time import sleep

from i2c import BNO08X_I2C
from bno08x import *

from machine import I2C, Pin
from utime import ticks_ms, sleep_us

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("===========================")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 250)

bno.print_report_period()
print("\nBNO08x sensors enabled")

while True:
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f} m/s²")
    # Notice Gravity acceleration is down