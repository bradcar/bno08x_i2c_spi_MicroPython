# test_reports_spi.py
#
# BNO08x MicroPython UART Test
#
# Raw device reports: raw_accelerometer, raw_magnetic, raw_gyro
#  - raw_accelerometer, raw_magnetic -  return 3 values and time_stamp
#  - raw_gyro - returnn 3 values, Celsius, and time_stamp
#
# note: timestamp is not clearly described in Ceva documentation
#
# Enabling reports at default frequencies
# Raw sensors will automatically enable additional required reports

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

print(uart)  # baudrate 3000000 required
print("Start")
print("====================================\n")

bno.raw_acceleration.enable()
bno.raw_magnetic.enable()
bno.raw_gyro.enable()

# sensor default frequencies
bno.print_report_period()

while True:
    # Required each loop to refresh sensor data
    bno.update_sensors
        
    accel_x, accel_y, accel_z, ts_us = bno.raw_acceleration
    print(f"\nRaw Acceleration:  X: {accel_x:#06x}  Y: {accel_y:#06x}  Z: {accel_z:#06x} {ts_us=}")

    mag_x, mag_y, mag_z, ts_us = bno.raw_magnetic
    print(f"Raw Magnetometer:  X: {mag_x:#06x}  Y: {mag_y:#06x}  Z: {mag_z:#06x} {ts_us=}")

    gyro_x, gyro_y, gyro_z, celsius, ts_us = bno.raw_gyro
    print(f"Raw Gyroscope:     X: {gyro_x:#06x}  Y: {gyro_y:#06x}  Z: {gyro_z:#06x} {celsius=} {ts_us=}")
