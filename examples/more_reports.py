# more_reports.py
#
# BNO08x Micropython I2C example program
#

from time import sleep

from bno08x import BNO_REPORT_STEP_COUNTER, BNO_REPORT_STABILITY_CLASSIFIER, BNO_REPORT_ACTIVITY_CLASSIFIER, \
    BNO_REPORT_SHAKE_DETECTOR
from i2c import *
from machine import I2C, Pin

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)
bno = BNO08X_I2C(i2c0, address=0x4B, debug=False)

bno.enable_feature(BNO_REPORT_STEP_COUNTER)
bno.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_ACTIVITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_SHAKE_DETECTOR)
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
