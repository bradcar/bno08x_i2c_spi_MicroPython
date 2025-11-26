# test_reports_spi.py
#
# BNO08x MicroPython UART Test
#
# UART interface: Test simple sensor report for acceleration

from time import sleep

from uart import BNO08X_UART
from bno08x import *

from machine import UART, Pin
from utime import ticks_ms, sleep_us

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset

uart = UART(1, baudrate=3_000_000, tx=Pin(8), rx=Pin(9), timeout=500)
bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("Start")
print("====================================")

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