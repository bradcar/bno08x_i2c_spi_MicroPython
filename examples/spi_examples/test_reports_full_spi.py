# test_reports_full_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test common sensor reports:
# acceleration.full, magnetic.full, gryo.full, quaternion.full, quaternion.euler_full
# full reports with accuracy and timestamps. Timestamps are microseconds (ms) in float from
# start of sensor. The function bno.bno_start_diff(ticks) will return ms between ticks & sensor start.

from time import sleep

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO sck 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=3_000_000)

print("Start")
bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)
print(spi)  # Notice polarity=1, phase=1 for bno08x
print("====================================\n")

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
    print(f"\nsystem timestamp {ticks_ms()=}, seconds from bno start: {ms_since_sensor_start/1000.0:.3f} sec")

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
