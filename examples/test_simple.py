# test_simple.py
#
# BNO08x MicroPython I2C Test
#
# I2C interface: Test simple sensor report for acceleration
#
# Reports requested at 100 Hz, sensor chooses 125Hz (8 msec between reports)
# It prints so much data that your console may not keep up.

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

address = 0x4b
i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("all I2C devices found:", [hex(d) for d in i2c0.scan()])

print("Start")
print("====================================\n")

bno.acceleration.enable(100)
bno.print_report_period()

while True:
    # Update required each loop to check if any sensor updated, print sensor data only if sensor is updated
    bno.update_sensors()

    if bno.acceleration.updated:
        accel_x, accel_y, accel_z = bno.acceleration
        print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}")

        # Notice Gravity acceleration downwards (~9.8 m/s²)
