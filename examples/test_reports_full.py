# test_reports_full.py
#
# BNO08x MicroPython I2C Test
#
# I2C interface: Test common sensor reports:
# acceleration.full, magnetic.full, gryo.full, quaternion.full, quaternion.euler_full
# full reports with accuracy and timestamps
# notice: with slow report frequency, the report can be from last time at not at this time

from time import sleep

from bno08x import *
from i2c import BNO08X_I2C
from machine import I2C, Pin
from utime import ticks_ms

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(15, Pin.OUT)  # BNO sensor (RST)

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)
bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=reset_pin, int_pin=int_pin, debug=False)

print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("Start")
print("===========================")

# with 0.25s sleep in loop, we request 4Hz reports (~0.25s)
bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_MAGNETOMETER, 20)
bno.enable_feature(BNO_REPORT_GYROSCOPE, 20)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, 20)

# sensor provides frequencies close to what was requested
bno.print_report_period()
print("\nBNO08x sensors enabled")

while True:
    sleep(.25)
        
    ms_since_sensor_start = bno.bno_start_diff(ticks_ms())
    print(f"\nsystem {ticks_ms()=},",
        f"seconds from BNO start: {ms_since_sensor_start/1000.0:.4f} s",
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
