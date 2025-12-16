# test_simple.py
#
# BNO08x MicroPython I2C Test
#
# I2C interface: Test simple sensor report for acceleration
#
# Reports enabled at 250Hz (4 msec between reports)
# It prints so much data that your console may not keep up.

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("Start")
print("====================================\n")

bno.acceleration.enable(100)

bno.print_report_period()

while True:
    # required to get data from enabled sensors
    bno.update_sensors()

    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f} m/s²")
    # Notice Gravity acceleration downwards (~9.8 m/s²)
