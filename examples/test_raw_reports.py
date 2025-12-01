# test_raw_reports.py
#
# BNO08x Micropython I2C example program
# Raw device reports: raw_accelerometer, raw_magnetic, raw_gyro
#  - raw_accelerometer, raw_magnetic -  return 3 values and time_stamp
#  - raw_gyro - returnn 3 values, Celsius, and time_stamp
# note: timestamp is not clearly described in Ceva documentation

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin)

print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("Start")
print("===========================")

bno.enable_feature(BNO_REPORT_RAW_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_RAW_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_RAW_GYROSCOPE)

# sensor default frequencies
bno.print_report_period()
print("\nBNO08x sensors enabled")

while True:
    accel_x, accel_y, accel_z, ts_us = bno.raw_acceleration
    print(f"\nRaw Acceleration:  X: {accel_x:#06x}  Y: {accel_y:#06x}  Z: {accel_z:#06x} {ts_us=}")

    mag_x, mag_y, mag_z, ts_us = bno.raw_magnetic
    print(f"Raw Magnetometer:  X: {mag_x:#06x}  Y: {mag_y:#06x}  Z: {mag_z:#06x} {ts_us=}")

    gyro_x, gyro_y, gyro_z, celsius, ts_us = bno.raw_gyro
    print(f"Raw Gyroscope:     X: {gyro_x:#06x}  Y: {gyro_y:#06x}  Z: {gyro_z:#06x} {celsius=} {ts_us=}")
