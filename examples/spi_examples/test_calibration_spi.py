# test_calibration_spi.py
#
# BNO08x MicroPython SPI Test
#
# Calibration of three main sensors.
# see README.md "Basic User Sensor Calibration Procedure" for recommened sensor movements

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

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

print("Start")
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)
print(spi)  # Notice polarity=1, phase=1 for bno08x
print("====================================\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 5)
bno.enable_feature(BNO_REPORT_MAGNETOMETER, 5)
bno.enable_feature(BNO_REPORT_GYROSCOPE, 5)

bno.print_report_period()
print("\nBNO08x sensors enabled")

GOOD_SECONDS = 5
start_good = None
calibration_good = False
status = ""

# Begin calibration
bno.begin_calibration
# Wait sensor to be ready to calibrate
bno.calibration_status

print(f"\nCalibration: Continue for {GOOD_SECONDS} secs of \"Medium Accuracy\" to \"High Accuracy\"")
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
        if start_good:
            print("\nlost calibration, resetting timer\n")
        status = "low accuracy, move sensor"
        calibration_good = False

    print(f"Accuracy: {accel_accuracy=}, {mag_accuracy=}, {gyro_accuracy=}\t{status}")

    if calibration_good:
        if start_good is None:
            start_good = ticks_ms()
            print(f"\nCalibration now good on all sensors. Start {GOOD_SECONDS}-second timer...\n")
        else:
            elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
            if elapsed >= GOOD_SECONDS:
                print(f"\n*** Calibration stable for {GOOD_SECONDS} secs")
                break
    else:
        start_good = None

# Exited loop
bno.save_calibration_data()
print("*** Calibration saved !")
