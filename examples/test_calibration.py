# test_calibration.py
#
# BNO08x Micropython I2C example program
# Calibration of three main sensors.
# see README.md "Basic User Sensor Calibration Procedure" for recommened sensor movements

from time import sleep

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("Start")
print("====================================\n")

bno.acceleration.enable(20)
bno.magnetic.enable(20)
bno.gyro.enable(20)

bno.print_report_period()

good_before_save = 5
start_good = None
calibration_good = False
status = ""

# Begin calibration
bno.begin_calibration

# Wait sensor to be ready to calibrate
bno.calibration_status

print(f"\nCalibration: Continue for {good_before_save} secs of \"Medium Accuracy\" to \"High Accuracy\"\n")


last_print = ticks_ms()
start_good = None

while True:
    bno.update_sensors() # Always update sensors, avoid using sleep

    # only print every .2 sec (200 ms)
    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    _, _, _, accel_accuracy, _ = bno.acceleration.full
    _, _, _, mag_accuracy, _ = bno.magnetic.full
    _, _, _, gyro_accuracy, _ = bno.gyro.full

    if all(x >= 2 for x in (accel_accuracy, mag_accuracy, gyro_accuracy)):
        status = "All Sensors >= 2"
        calibration_good = True
    else:
        if start_good is not None:
            print("\nlost calibration, resetting timer\n")
        status = "low accuracy, move sensor"
        calibration_good = False

    print(f"Accuracy: acceleration={accel_accuracy}, magnetic={mag_accuracy}, gyro={gyro_accuracy}\t{status}")

    if calibration_good:
        if start_good is None:
            start_good = ticks_ms()
            print(f"\nCalibration >=2 on all sensors. Start {good_before_save}-second timer...\n")
        else:
            elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
            if elapsed >= good_before_save:
                print(f"\n*** Calibration stable for {good_before_save} secs\n")
                break
    else:
        start_good = None

# Exited loop
bno.save_calibration_data()
print("*** Calibration saved")
