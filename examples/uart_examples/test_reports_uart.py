# test_reports_uart.py
#
# BNO08x MicroPython UART Test
#
# UART interface: Test common sensor reports:
# acceleration, magnetic, gryoscope, quaternion, quaternion.euler
#
# Enabling reports at 4 Hz (~0.25 sec)
# sensor provides frequencies close to what was requested

from bno08x import *
from machine import UART, Pin
from uart import BNO08X_UART
from utime import ticks_ms

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT, value=1)  # BNO sensor (RST)

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9))

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print(uart)  # baudrate 3000000 required
print("Start")
print("====================================\n")

bno.acceleration.enable(4)
bno.magnetic.enable(4)
bno.gyro.enable(4)
bno.quaternion.enable(4)

# sensor provides frequencies close to what was requested
bno.print_report_period()

print("\nStart loop:")
while True:
    # Required each loop to refresh sensor data
    bno.update_sensors()

    print(f"\nsystem {ticks_ms()=}")

    accel_x, accel_y, accel_z = bno.acceleration
    print(f"\nAcceleration X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")

    mag_x, mag_y, mag_z = bno.magnetic
    print(f"Magnetometer X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT")

    gyro_x, gyro_y, gyro_z = bno.gyro
    print(f"Gyroscope    X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"Quaternion   I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.quaternion.euler
    print(f"Euler Angle: Roll {roll:+.3f}°  Pitch: {pitch:+.3f}°  Yaw: {yaw:+.3f}°  degrees")
