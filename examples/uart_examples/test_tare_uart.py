# test_tare_uart.py
#
# BNO08x MicroPython UART Test
#
# measure quaternion, use euler_conversion, tare the sensor, and show new orientation

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

print(uart)  # baudrate 3000000 required
print("Start")
print("===========================\n")

bno.quaternion.enable(100)

bno.print_report_period()

good_before_save = 5
start_good = None
calibration_good = False
status = ""

# show orientation for 9 seconds
for t in range(1, 9):
    bno.update_sensors

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

print("\n\n*** Starting Countdown timer for 5 seconds, then tare the sensor\n")
for t in range(5, 0, -1):
    bno.update_sensors

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

axis = 0x07  # tare all Axis (z, y, x)
bno.tare(axis, BNO_REPORT_ROTATION_VECTOR)

print(f"\n\n*** Tare the sensor axis=({hex(axis)})\n")

# show new orientation based on tare for 9 seconds
for t in range(1, 9):
    bno.update_sensors

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

# Exited loop
bno.save_tare_data
print("\n\t*** Tare saved")