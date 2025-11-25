# test_calibration.py
#
# TODO: BRC save_calibration_data works for i2c, but not spi
#
# BNO08x Micropython I2C example program
# Calibration of Magnetometer magnetic accuracy report

from time import sleep

from i2c import BNO08X_I2C
from bno08x import *

from machine import I2C, Pin
from utime import ticks_ms, ticks_diff

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("===========================")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 5)
bno.enable_feature(BNO_REPORT_MAGNETOMETER, 5)
bno.enable_feature(BNO_REPORT_GYROSCOPE, 5)

bno.print_report_period()
print("\nBNO08x sensors enabled")

GOOD_SECONDS = 3
start_good = None
calibration_good = False
status = ""

print(f"\nCalibration: continues for {GOOD_SECONDS} secs of \"Medium Accuracy\" to \"High Accuracy\"")
bno.begin_calibration
bno.calibration_status

while True:
    sleep(0.2)

    _, _, _, accel_accuracy, _ = bno.acceleration.full
    _, _, _, mag_accuracy, _ = bno.magnetic.full
    _, _, _, gyro_accuracy, _ = bno.gyro.full
    
    # Check calibration of all timers
    if all(x >= 2 for x in (accel_accuracy, mag_accuracy, gyro_accuracy)):
        status = "All Good !"
        calibration_good = True
    else:
        status = "low accuracy, move sensor"
        calibration_good = False
        
    print(f"Accuracy: {accel_accuracy=}, {mag_accuracy=}, {gyro_accuracy=}\t{status}")

    if calibration_good:
        if start_good is None:
            start_good = ticks_ms()
            print(f"\nCalibration now good on all sensors. Start {GOOD_SECONDS}-second timer...\n")
        else:
            elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
            # Exit if 3 seconds of all good calibration
            if elapsed >= GOOD_SECONDS:
                print(f"\n*** Calibration stable for {GOOD_SECONDS} secs")
                break  
    else:
        start_good = None

#Exited loop
bno.save_calibration_data()
print("*** Calibration saved !")