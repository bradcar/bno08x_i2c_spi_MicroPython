# test_tare_spi.py
#
# BNO08x MicroPython SPI Test
# measure quaternion, use euler_conversion, tare the sensor, and show new orientation

from bno08x import *
from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms, ticks_diff

int_pin = Pin(14, Pin.IN)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
print(spi) # baudrate=3000000 required

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)
print(spi)

print("Start")
print("===========================")

bno.quaternion.enable(100)
bno.print_report_period()

print("\n\n*** Starting Countdown timer for 10 seconds, then tare the sensor\n")
start = ticks_ms()
secs = 10
while secs > 0:
    bno.update_sensors()

    if ticks_diff(ticks_ms(), start) < 1000:
        continue

    qr, qi, qj, qk = bno.quaternion
    print(f"Quaternion  QR: {qr:+.3f}  QI: {qi:+.3f}  QJ: {qj:+.3f}  QK: {qk:+.3f}")
    yaw, pitch, roll = bno.euler_conversion(qr, qi, qj, qk)
    print(f"     Euler  Yaw: {yaw:+.1f}°   Pitch: {pitch:+.1f}°  Roll {roll:+.1f}° degrees")

    start = ticks_ms()
    secs -= 1

# Tare the orientation
axis = 0x07  # tare all Axis (z, y, x)
basis = 0  # Quaternion
bno.tare(axis, basis)

print(f"\n\n*** Tared the sensor axis=({hex(axis)}), basis={basis})\n")

# show the new orientation based on tare for 7 seconds
start = ticks_ms()
secs = 7
while secs > 0:
    bno.update_sensors()

    if ticks_diff(ticks_ms(), start) < 1000:
        continue

    qr, qi, qj, qk = bno.quaternion
    print(f"Quaternion   QR: {qr:+.3f}  QI: {qi:+.3f}  QJ: {qj:+.3f}  QK: {qk:+.3f}")
    yaw, pitch, roll = bno.euler_conversion(qr, qi, qj, qk)
    print(f"     Euler  Yaw: {yaw:+.1f}°   Pitch: {pitch:+.1f}°  Roll {roll:+.1f}° degrees")

    start = ticks_ms()
    secs -= 1

# Exited loop
bno.save_tare_data()
print("\n\t*** Tare saved (from 7 seconds ago)")
