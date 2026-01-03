# test_more_reports_uart.py
#
# BNO08x MicroPython UART Test
#
# Steps counted,  Stability classifier, Activity classifier

from bno08x import *
from machine import UART, Pin
from uart import BNO08X_UART
from utime import ticks_ms

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9))
print(uart)  # baudrate 3000000 required

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print("Start")
print("====================================\n")

bno.steps.enable()
bno.stability_classifier.enable()
bno.activity_classifier.enable()

bno.print_report_period()

last_print = ticks_ms()
print("\nStart loop:")
while True:
    # Required each loop to refresh sensor data
    bno.update_sensors()

    # print out results every 0.5 sec (500 ms)
    now = ticks_ms()
    if ticks_diff(now, last_print) >= 500:
        last_print = now

        print(f"\nTotal Steps detected: {bno.steps}")
        print(f"Stability classifier: {bno.stability_classifier}")

        activity, confidence = bno.activity_classifier
        print(f"Activity classifier: {activity}, confidence: {confidence}%")
