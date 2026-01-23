# test_raw_reports_spi.py
#
# BNO08x MicroPython SPI Test
# Raw device reports: raw_accelerometer, raw_magnetic, raw_gyro
#  - raw_accelerometer, raw_magnetic -  return 3 values and time_stamp
#  - raw_gyro - returnn 3 values, Celsius, and time_stamp
#
# note: timestamp is not clearly described in Ceva documentation
#
# Enabling reports at default frequencies
# Raw sensors will automatically enable additional required reports

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCI 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
print(spi) # baudrate=3000000 required

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)
print(spi)

print("Start")
print("====================================\n")

bno.raw_acceleration.enable()
bno.raw_magnetic.enable()
bno.raw_gyro.enable()

# sensor default frequencies
bno.print_report_period()

while True:
    # Update required each loop to check if any sensor updated, print sensor data (some or all may be old data)
    # see test_reports_full_spi.py, for example of only printing a sensor when it is updated
    bno.update_sensors()
        
    accel_x, accel_y, accel_z, ts_us = bno.raw_acceleration
    print(f"\nRaw Acceleration:  X: {accel_x:#06x}  Y: {accel_y:#06x}  Z: {accel_z:#06x} {ts_us=}")

    mag_x, mag_y, mag_z, ts_us = bno.raw_magnetic
    print(f"Raw Magnetometer:  X: {mag_x:#06x}  Y: {mag_y:#06x}  Z: {mag_z:#06x} {ts_us=}")

    gyro_x, gyro_y, gyro_z, celsius, ts_us = bno.raw_gyro
    print(f"Raw Gyroscope:     X: {gyro_x:#06x}  Y: {gyro_y:#06x}  Z: {gyro_z:#06x} {celsius=} {ts_us=}")
