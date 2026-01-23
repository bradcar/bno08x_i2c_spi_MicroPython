# test_more_reports_spi.py
#
# BNO08x MicroPython SPI Test
# Steps counted,  Stability classifier, Activity classifier

from time import ticks_ms, ticks_diff

from bno08x import *
from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
print(spi) # baudrate=3000000 required

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)
print(spi)

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
