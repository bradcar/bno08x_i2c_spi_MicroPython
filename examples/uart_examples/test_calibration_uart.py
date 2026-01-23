# test_calibration_uart.py
#
# BNO08x MicroPython UART Test
# Calibration of three main sensors.
# see README.md "Basic User Sensor Calibration Procedure" for recommened sensor movementsn

from bno08x import *
from machine import UART, Pin
from uart import BNO08X_UART
from utime import ticks_ms

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9))
print(uart)  # baudrate 3000000 required

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)


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
