# test_reports_full_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test common sensor reports:
# acceleration.full, magnetic.full, gryo.full, quaternion.full, quaternion.euler_full
# full reports with accuracy and timestamps. Timestamps are microseconds (ms) in float from
# start of sensor. The function bno.bno_start_diff(ticks) will return ms between ticks & sensor start.
#
# Enabling reports at high frequencies 100Hz, some sensors can not run at this rate
# sensor provides frequencies close to what was requested
#
# Conditionals added to only print data if sensor report updated.
#
# any prints slow processing, max period is ~18ms even though report at 8ms to 10ms.

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)

print(spi) # baudrate=3000000 required
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
    if bno.update_sensors() > 0:    
        ms_since_sensor_start = bno.bno_start_diff(ticks_ms())
        print(f"\nsystem {ticks_ms()=},",
            f"time from BNO start: {ms_since_sensor_start/1000.0:.3f} s",
            f"({ms_since_sensor_start:.0f} ms)")
    
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
