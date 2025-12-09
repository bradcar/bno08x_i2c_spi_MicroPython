# test_reports_full_uart.py
#
# BNO08x MicroPython UART Test
#
# UART interface: Test common sensor reports:
# acceleration.full, magnetic.full, gryo.full, quaternion.full, quaternion.euler_full
# full reports with accuracy and timestamps
# notice: with slow report frequency, the report can be from last time at not at this time

from time import sleep

from uart import BNO08X_UART
from bno08x import *

from machine import UART, Pin
from utime import ticks_ms, sleep_us

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT, value=1)  # BNO sensor (RST)

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9), timeout=500)
bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print(uart)  # baudrate 3000000 required
print("Start")
print("====================================\n")

bno.acceleration.enable(100)
bno.magnetic.enable(100)
bno.gyro.enable(100)
bno.quaternion.enable(100)

bno.print_report_period()

print("\nStart loop:")
while True:
    # Required each loop to refresh sensor data
    bno.update_sensors
    
    ms_since_sensor_start = bno.bno_start_diff(ticks_ms())
    print(f"\nsystem {ticks_ms()=},",
        f"time from BNO start: {ms_since_sensor_start/1000.0:.3f} s",
        f"({ms_since_sensor_start:.0f} ms)")
    
    accel_x, accel_y, accel_z, acc, ts_ms = bno.acceleration.full
    print(f"\nAcceleration X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")
    print(f"Acceleration: accuracy={acc}, {ts_ms=:.1f}")

    mag_x, mag_y, mag_z, acc, ts_ms = bno.magnetic.full
    print(f"Magnetometer X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT ms")
    print(f"Magnetometer: accuracy={acc}, {ts_ms=:.1f}")

    gyro_x, gyro_y, gyro_z, acc, ts_ms = bno.gyro.full
    print(f"Gyroscope    X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")
    print(f"Gyroscope: accuracy={acc}, {ts_ms=:.1f}")

    quat_i, quat_j, quat_k, quat_real, acc, ts_ms = bno.quaternion.full
    print(f"Quaternion   I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    print(f"Quaternion: accuracy={acc}, {ts_ms=:.1f}")

    roll, pitch, yaw, acc, ts_ms = bno.quaternion.euler_full
    print(f"Euler Angle: Roll {roll:+.3f}°  Pitch: {pitch:+.3f}°  Yaw: {yaw:+.3f}°  degrees")
    print(f"Euler Angle: accuracy={acc}, {ts_ms=:.1f}")
