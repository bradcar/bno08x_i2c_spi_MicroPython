# BNO08x MicroPython I2C Test
#
# I2C interface: Test common sensor reports  

from time import sleep

from i2c import BNO08X_I2C
from bno08x import *

from machine import I2C, Pin
from utime import ticks_ms, sleep_us

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("====================================\n")

bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

# with 0.25s sleep in loop, only selected 4Hz reports requested
bno.enable_feature(BNO_REPORT_ACCELEROMETER, 4)
bno.enable_feature(BNO_REPORT_MAGNETOMETER, 4)
bno.enable_feature(BNO_REPORT_GYROSCOPE, 4)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, 4)

bno.print_report_period()
print("\nBNO08x sensors enabled\n")

i = 0
start = ticks_ms()

while True:
    sleep (.25)
    accel_x, accel_y, accel_z, acc, ts_us = bno.acceleration.full
    print(f"\nAccel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")
    mag_x, mag_y, mag_z = bno.magnetic
    print(f"Magnetometer  X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT ms")
    gyro_x, gyro_y, gyro_z = bno.gyro
    print(f"Gyroscope     X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"Rot Vect Quat I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.quaternion.euler
    print(f"Euler Roll: {roll:+.3f}°  Pitch: {pitch:+.3f}°  Yaw: {yaw:+.3f}°  degrees")
