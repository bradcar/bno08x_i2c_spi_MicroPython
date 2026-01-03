# test_reports_full_uart.py
  #
# BNO08x MicroPython UART Test
#
# UART interface: Test common sensor reports:
# acceleration.full, magnetic.full, gryo.full, quaternion.full, quaternion.euler_full
# full reports with accuracy and timestamps
#
# Enabling reports at high frequencies 100Hz, some sensors can not run at this rate
# sensor provides frequencies close to what was requested
#
# Conditionals added to only print data if sensor report updated.
#
# any prints slow processing, max period is ~18ms even though report at 8ms to 10ms.

from bno08x import *
from machine import UART, Pin
from uart import BNO08X_UART
from utime import ticks_ms

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT, value=1)  # BNO sensor (RST)

uart = UART(1, baudrate=3000000, tx=Pin(8), rx=Pin(9), rxbuf=1024)
print(uart)  # baudrate 3000000 required

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print("Start")
print("====================================\n")

bno.acceleration.enable(20)
bno.magnetic.enable(20)
bno.gyro.enable(20)
bno.quaternion.enable(20)
bno.print_report_period()

print("\nStart loop:")
while True:
    
    # Update required each loop to check if any sensor updated, print timestamp if any sensor was updated
    if bno.update_sensors() > 0:    
        ms_since_sensor_start = bno.bno_start_diff(ticks_ms())
        print(f"\nsystem {ticks_ms()=},",
            f"time from BNO start: {ms_since_sensor_start/1000.0:.3f} s",
            f"({ms_since_sensor_start:.0f} ms)")
    
    # Only print sensor report if it has been updated since last loop
    
    if bno.acceleration.updated:
        accel_x, accel_y, accel_z, acc, ts_ms = bno.acceleration.full
        print(f"\nAcceleration X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")
        print(f"Acceleration: accuracy={acc}, {ts_ms=:.1f}")

    if bno.magnetic.updated:
        mag_x, mag_y, mag_z, acc, ts_ms = bno.magnetic.full
        print(f"Magnetometer X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT ms")
        print(f"Magnetometer: accuracy={acc}, {ts_ms=:.1f}")

    if bno.gyro.updated:
        gyro_x, gyro_y, gyro_z, acc, ts_ms = bno.gyro.full
        print(f"Gyroscope    X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")
        print(f"Gyroscope: accuracy={acc}, {ts_ms=:.1f}")

    if bno.quaternion.updated:
        quat_i, quat_j, quat_k, quat_real, acc, ts_ms = bno.quaternion.full
        print(f"Quaternion   I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
        print(f"Quaternion: accuracy={acc}, {ts_ms=:.1f}")

        roll, pitch, yaw, acc, ts_ms = bno.quaternion.euler_full
        print(f"Euler Angle: Roll {roll:+.3f}°  Pitch: {pitch:+.3f}°  Yaw: {yaw:+.3f}°  degrees")
        print(f"Euler Angle: accuracy={acc}, {ts_ms=:.1f}")
