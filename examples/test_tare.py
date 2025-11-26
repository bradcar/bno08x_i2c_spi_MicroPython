# test_tare.py
#
# BNO08x Micropython I2C example program
# measure quaternion, use euler_conversion, tare the sensor, and show inew orientation

from time import sleep

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("===========================")

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, 20)

bno.print_report_period()
print("\nBNO08x sensors enabled")

GOOD_SECONDS = 5
start_good = None
calibration_good = False
status = ""

# show orientation for 9 seconds
for t in range(1, 9):
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

print("\n\n*** Starting Countdown timer for 5 seconds, then tare the sensor\n")
for t in range(5, 0, -1):
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

axis = 0x07  # tare all Axis (z, y, x)
bno.tare(axis, BNO_REPORT_ROTATION_VECTOR)

print(f"\n\n*** Tare the sensor ({hex(axis)}, {hex(BNO_REPORT_ROTATION_VECTOR)})\n")

# show new orientation based on tare for 9 seconds
for t in range(1, 9):
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

# Exited loop
bno.save_tare_data
print("\n\t*** Tare saved to flash.")
