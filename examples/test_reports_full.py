# test_reports_full.py
#
# BNO08x MicroPython I2C Test
#
# I2C interface: Test common sensor reports:
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
from i2c import BNO08X_I2C
from machine import I2C, Pin
from utime import ticks_ms

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

address = 0x4b
i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("I2C devices found:", [hex(d) for d in i2c0.scan()])

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
              f"time from BNO start: {ms_since_sensor_start / 1000.0:.3f} s",
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
        qr, qi, qj, qk, acc, ts_ms = bno.quaternion.full
        print(f"Quaternion  Real: {qr:+.3f} I: {qi:+.3f}  J: {qj:+.3f}  K: {qk:+.3f}")
        print(f"Quaternion: accuracy={acc}, {ts_ms=:.1f}")

        yaw, pitch, roll, acc, ts_ms = bno.quaternion.euler_full
        print(f"Euler Angle: Yaw: {yaw:+.3f}°   Pitch: {pitch:+.3f}°  Roll {roll:+.3f}° degrees")
        print(f"Euler Angle: accuracy={acc}, {ts_ms=:.1f}")