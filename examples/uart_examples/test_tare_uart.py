# test_tare_uart.py
#
# BNO08x MicroPython UART Test
#
# measure quaternion, use euler_conversion, tare the sensor, and show new orientation

from bno08x import *
from machine import UART, Pin
from uart import BNO08X_UART
from utime import ticks_ms, ticks_diff

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset

uart = UART(1, baudrate=3_000_000, tx=Pin(8), rx=Pin(9))

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print(uart)  # baudrate 3000000 required
print("Start")
print("===========================")

bno.quaternion.enable(100)
bno.print_report_period()

print("\n\n*** Starting Countdown timer for 10 seconds, then tare the sensor\n")
start = ticks_ms()
secs = 10
while secs > 0:
    bno.update_sensors()

    if ticks_diff(ticks_ms(), start) < 1000:
        continue

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={secs}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")

    start = ticks_ms()
    secs -= 1

# Tare the orientation
axis = 0x07  # tare all Axis (z, y, x)
basis = 0  # Quaternion
bno.tare(axis, basis)

print(f"\n\n*** Tared the sensor axis=({hex(axis)}), basis={basis})\n")

# show the new orientation based on tare for 7 seconds
start = ticks_ms()
secs = 7
while secs > 0:
    bno.update_sensors()

    if ticks_diff(ticks_ms(), start) < 1000:
        continue

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={secs}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")

    start = ticks_ms()
    secs -= 1

# Exited loop
bno.save_tare_data()
print("\n\t*** Tare saved (from 7 seconds ago)")
