# test_simple_uart.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test simple sensor report for acceleration
#
# Reports requested at 100 Hz, sensor chooses 125Hz (8 msec between reports)
# It prints so much data that your console may not keep up.

from bno08x import *
from machine import UART, Pin
from uart import BNO08X_UART

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9))
print(uart)  # baudrate 3000000 required

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print("Start")
print("====================================\n")

bno.acceleration.enable(100)
bno.print_report_period()

while True:
    # Update required to refresh sensor data
    bno.update_sensors()

    if bno.acceleration.updated:
        accel_x, accel_y, accel_z = bno.acceleration
        print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}")

    # Notice Gravity acceleration downwards (~9.8 m/s²)
