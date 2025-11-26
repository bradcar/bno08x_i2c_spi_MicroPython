# test_simple_uart.py
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

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 250)

bno.print_report_period()
print("\nBNO08x sensors enabled")

while True:
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f} m/s²")
    # Notice Gravity acceleration downwards (~9.8 m/s²)
