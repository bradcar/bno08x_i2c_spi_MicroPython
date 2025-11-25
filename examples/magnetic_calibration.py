# magnetic_calibration.py
#
# TODO: BRC save_calibration_data works for i2c, but not spi
#
# BNO08x Micropython I2C example program
# Calibration of Magnetometer magnetic accuracy report

from time import sleep

from i2c import BNO08X_I2C
from bno08x import *

from machine import I2C, Pin
from utime import ticks_ms, sleep_us

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("===========================")

bno.enable_feature(BNO_REPORT_MAGNETOMETER, 20)

bno.print_report_period()
print("BNO08x sensors enabled")

print("Begin calibration: continue until 5 secs of constant \"Medium Accuracy\" to \"High Accuracy\"")
bno.begin_calibration()
calibration_good_at = None
save_pending = False

while True:
    sleep(0.2)
    mag_x, mag_y, mag_z, accuracy, ts= bno.magnetic.full
    print(f"\nMagnetometer:  X: {mag_x:.4f}  Y: {mag_y:.4f}  Z: {mag_z:.4f}  uT  {accuracy=}")

    # print accuracy value and string
  
    print(f"Magnetic Accuracy: = {accuracy}")
    now = ticks_ms()

    # Check when calibration becomes "good"
    if not calibration_good_at and accuracy >= 2:
        calibration_good_at = now
        print(f"Calibration good at: {now / 1000.0:.3f}s")

    # after calibration has been good for 5 seconds, save calibration
    if calibration_good_at and ticks_diff(now, calibration_good_at) > 5000:
        if not save_pending:
            print("\n*** Calibration stable for 5 sec, saving calibration")
            bno.save_calibration_data()
            save_pending = True
        break

    # Reset timer if calibration drops
    if accuracy < 2:
        if calibration_good_at:
            print("--- Calibration lost, restarting timer...")
        calibration_good_at = None
        save_pending = False

print("calibration done")