# test_more_reports_uart.py
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

bno.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_ACTIVITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_SHAKE_DETECTOR)
bno.enable_feature(BNO_REPORT_STEP_COUNTER)


print("BNO08x reports enabled\n")
bno.print_report_period()
print()

while True:
    sleep(0.1)

    print(f"\nTotal Steps detected: {bno.steps=}")
    print(f"Stability classification: {bno.stability_classification=}")

    activity_classification = bno.activity_classification
    most_likely = activity_classification["most_likely"]
    confidence = activity_classification.get(most_likely, 0)  # safe default
    print(f"Activity classification: {most_likely}, confidence: {confidence}/100")

    print("sleep for 0.5 sec, then test shake")
    sleep(0.5)
    if bno.shake:
        print("Shake Detected! \n")
