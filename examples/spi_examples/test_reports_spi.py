# test_reports_spi.py
#
# BNO08x MicroPython SPI Test
#
# SPI interface: Test common sensor reports:
# acceleration, magnetic, gryoscope, quaternion, quaternion.euler
#
# Enabling reports at 4 Hz (~0.25 sec)
# sensor provides frequencies close to what was requested

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

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)

print(spi) # baudrate=3000000 required
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
    bno.update_sensors

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
