# more_reports.py
#
# BNO08x Micropython additioanl report SPI example program

from bno08x import *
from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms
from time import sleep

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# spi0_RX = Pin(16, Pin.IN)  # spi0_RX, RES (MISO) - connected to BNO SO (POCI)
cs = Pin(17, Pin.OUT, value=1)  # cs for SPI
# sck = Pin(18, Pin.OUT, value=0)  # sck for SPI
# spi0_TX = Pin(19, Pin.OUT, value=0)  # spi0_TX (MOSI) - connected to BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # Wakes BNO to enable INT response

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=1_000_000)

print("Start")
bno = BNO08X_SPI(spi, cs, reset_pin, int_pin, wake_pin, debug=True)
print(spi)  # polarity=1, phase=1 for bno08x
print("====================================\n")

bno.enable_feature(BNO_REPORT_STEP_COUNTER)
bno.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_ACTIVITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_SHAKE_DETECTOR)
print("BNO08x reports enabled\n")
bno.print_report_period()

while True:
    sleep(0.1)

    print(f"\nTotal Steps detected: {bno.steps=}")

    print(f"Stability classification: {bno.stability_classification=}")

    activity_classification = bno.activity_classification
    most_likely = activity_classification["most_likely"]
    confidence = activity_classification.get(most_likely, 0)  # safe default
    print(f"Activity classification: {most_likely}, confidence: {confidence}/100")

    sleep(0.5)
    if bno.shake:
        print("Shake Detected! \n")
