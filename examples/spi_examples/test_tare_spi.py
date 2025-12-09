# test_tare_spi.py
#
# BNO08x MicroPython SPI Test
# measure quaternion, use euler_conversion, tare the sensor, and show new orientation

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

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin, debug=False)

print(spi)
print("Start")
print("===========================\n")

bno.quaternion.enable(100)

bno.print_report_period()

good_before_save = 5
start_good = None
calibration_good = False
status = ""

# show orientation for 9 seconds
for t in range(1, 9):
    bno.update_sensors

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

print("\n\n*** Starting Countdown timer for 5 seconds, then tare the sensor\n")
for t in range(5, 0, -1):
    bno.update_sensors

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

axis = 0x07  # tare all Axis (z, y, x)
bno.tare(axis, BNO_REPORT_ROTATION_VECTOR)

print(f"\n\n*** Tare the sensor axis=({hex(axis)})\n")

# show new orientation based on tare for 9 seconds
for t in range(1, 9):
    bno.update_sensors

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"\nt={t}: Quaternion:  I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.euler_conversion(quat_i, quat_j, quat_k, quat_real)
    print(f"     Euler Angle: Roll {roll:+.1f}°  Pitch: {pitch:+.1f}°  Yaw: {yaw:+.1f}°  degrees")
    sleep(1)

# Exited loop
bno.save_tare_data
print("\n\t*** Tare saved")