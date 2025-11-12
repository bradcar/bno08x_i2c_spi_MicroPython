# magnetic_calibration.py
#
# TODO: BRC save_calibration_data works for i2c, but not spi
#
# BNO08x Micropython I2C example program
# Calibration of Magnetometer magnetic accuracy report

from time import sleep

from i2c import *
from bno08x import BNO_REPORT_MAGNETOMETER, REPORT_ACCURACY_STATUS

from machine import I2C, Pin
from utime import ticks_ms, ticks_diff

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)
bno = BNO08X_I2C(i2c0, address=0x4B, debug=False)

bno.enable_feature(BNO_REPORT_MAGNETOMETER, 20)
print("BNO08x reports enabled\n")
bno.print_report_period()
print()

print("Begin calibration: continue until 5 secs of constant \"Medium Accuracy\" to \"High Accuracy\"")
bno.begin_calibration()
calibration_good_at = None
save_pending = False

while True:
    sleep(0.2)
    mag_x, mag_y, mag_z = bno.magnetic
    print(f"\nMagnetometer:  X: {mag_x:.4f}  Y: {mag_y:.4f}  Z: {mag_z:.4f} uT")

    # print accuracy value and string
    calibration_status = bno.calibration_status
    print(f"Mag Calibration: \"{REPORT_ACCURACY_STATUS[calibration_status]}\" = {calibration_status}")
    now = ticks_ms()

    # Check when calibration becomes "good"
    if not calibration_good_at and calibration_status >= 2:
        calibration_good_at = now
        print(f"Calibration good at: {now / 1000.0:.3f}s")

    # If calibration stays good for 5 seconds, trigger save
    if calibration_good_at and ticks_diff(now, calibration_good_at) > 5000:
        if not save_pending:
            print("\n*** Calibration stable for 5 sec, saving calibration")
            bno.save_calibration_data()
            save_pending = True
        break

    # Reset timer if calibration drops
    if calibration_status < 2:
        if calibration_good_at:
            print("--- Calibration lost, restarting timer...")
        calibration_good_at = None
        save_pending = False

print("calibration done")