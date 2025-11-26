# test_calibration_uart.py
#
# BNO08x MicroPython UART Test
#
# UART interface: Test sensor calibration

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

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 10)
bno.enable_feature(BNO_REPORT_MAGNETOMETER, 10)
bno.enable_feature(BNO_REPORT_GYROSCOPE, 10)

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

#Exited loop
bno.save_calibration_data()
print("*** Calibration saved !")
