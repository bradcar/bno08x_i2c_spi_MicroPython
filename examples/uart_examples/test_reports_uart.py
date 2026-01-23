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
int_pin = Pin(14, Pin.IN)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT, value=1)  # BNO sensor (RST)

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9))
print(uart)  # baudrate 3000000 required

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print("Start")
print("====================================\n")

bno.acceleration.enable(5)
bno.magnetic.enable(5)
bno.gyro.enable(5)
bno.quaternion.enable(5)

# sensor provides frequencies close to what was requested
bno.print_report_period()

print("\nStart loop:")
while True:
    # Update required each loop to check if any sensor updated, print sensor data (some or all may be old data)
    # see test_reports_full_spi.py, for example of only printing a sensor when it is updated
    bno.update_sensors()

    print(f"\nsystem {ticks_ms()=}")

    accel_x, accel_y, accel_z = bno.acceleration
    print(f"\nAcceleration X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")

    mag_x, mag_y, mag_z = bno.magnetic
    print(f"Magnetometer X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT")

    gyro_x, gyro_y, gyro_z = bno.gyro
    print(f"Gyroscope    X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")

    qr, qi, qj, qk = bno.quaternion
    print(f"Quaternion   QR: {qr:+.3f}  QI: {qi:+.3f}  QJ: {qj:+.3f}  QK: {qk:+.3f}")

    yaw, pitch, roll = bno.quaternion.euler
    print(f"Euler Angle: Yaw: {yaw:+.3f}°   Pitch: {pitch:+.3f}°  Roll {roll:+.3f}° degrees")
