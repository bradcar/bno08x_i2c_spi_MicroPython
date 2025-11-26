# test_more_reports_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test simple sensor report for acceleration

from time import sleep

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO sck 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=3_000_000)

print("Start")
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)
print(spi)  # Notice polarity=1, phase=1 for bno08x
print("====================================\n")

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
