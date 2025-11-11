# raw_sensors.py
#
# BNO08x Micropython I2C example program
# Raw device reports: Raw_Accelerometer, Raw_Magnetometer, Raw_Gyroscope
# acc_raw amd mag_raw return 3 values and time_stamp
# gyro_raw return 3 values, Celsius, and time_stamp

from bno08x import *
from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# spi0_RX = Pin(16, Pin.IN)  # spi0_RX, RES (MISO) - connected to BNO SO (POCI)
cs = Pin(17, Pin.OUT, value=1)  # cs for SPI
# sck = Pin(18, Pin.OUT, value=0)  # sck for SPI
# spi0_TX = Pin(19, Pin.OUT, value=0)  # spi0_TX (MOSI) - connected to BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # Wakes BNO to enable INT response

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=3_000_000)

print("Start")
bno = BNO08X_SPI(spi, cs, reset_pin, int_pin, wake_pin, debug=False)
print(spi)  # polarity=1, phase=1 for bno08x
print("====================================\n")

bno.enable_feature(BNO_REPORT_RAW_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_RAW_MAGNETOMETER, 20)
bno.enable_feature(BNO_REPORT_RAW_GYROSCOPE, 20)
print("BNO08x sensors enabled\n")
bno.print_report_period()

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

while True:
    cpt += 1
    print("\ncpt", cpt)

    accel_x, accel_y, accel_z, time_stamp = bno.raw_acceleration
    print(f"Raw Acceleration:  X: {accel_x:#06x}  Y: {accel_y:#06x}  Z: {accel_z:#06x}  {time_stamp=}")

    mag_x, mag_y, mag_z, time_stamp = bno.raw_magnetic
    print(f"Raw Magnetometer:  X: {mag_x:#06x}  Y: {mag_y:#06x}  Z: {mag_z:#06x}  {time_stamp=}")

    gyro_x, gyro_y, gyro_z, celsius, time_stamp = bno.raw_gyro
    print(f"Raw Gyroscope:     X: {gyro_x:#06x}  Y: {gyro_y:#06x}  Z: {gyro_z:#06x}  {celsius=}  {time_stamp=}")

    print("===================================")
    timer = ticks_ms()
    average_delay = (timer - timer_origin) / cpt
    print(f"average delay times (ms) : {average_delay:.1f}")
    print("===================================")
