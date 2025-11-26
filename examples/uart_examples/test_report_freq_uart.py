# test_report_freq_uart.py
#
# BNO08x MicroPython SPI Test
#
# This program set up an SPI connection to the BNO08x sensor

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

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 60)
#bno.enable_feature(BNO_REPORT_MAGNETOMETER, 100)

bno.print_report_period()
print("BNO08x sensors enabled\n")

cpt = 0

start = ticks_ms()
last_acc_ms = 0
last_mag_ms = 0
running_sum = 0.0

# ignore first call timing
accel_x, accel_y, accel_z, acc, ts_us = bno.acceleration.full
last_acc_ms = ts_us/1000.0

while True:
    accel_x, accel_y, accel_z, acc, ts_us = bno.acceleration.full
#    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f} m/s², Acc={acc}, {ts_us/1000.0 - last_acc_ms:.1f} ms")
#    print(f"Accel {ts_us/1000.0 - last_acc_ms:.1f} ms")
    iter_time = ts_us/1000.0 - last_acc_ms
    running_sum += iter_time
    cpt +=1
    print(f"ave={running_sum/cpt:.1f}, current={iter_time:.1f}")
    # print(f"ave={running_sum/cpt:.1f}")


    last_acc_ms = ts_us/1000.0
    
#     mag_x, mag_y, mag_z, acc, ts_us = bno.magnetic.full
# #    print(f"Magnetometer  X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f} uT, Acc={acc}, {ts_us/1000.0 - last_mag_ms:.1f} ms")
#     print(f"Mag {ts_us/1000.0 - last_mag_ms:.1f} ms")
#     last_mag_ms = ts_us/1000.0
