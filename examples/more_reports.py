# more_reports.py
#
# BNO08x Micropython I2C example program
#

from time import sleep

from i2c import BNO08X_I2C
from bno08x import *
from machine import I2C, Pin

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("===========================")

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
