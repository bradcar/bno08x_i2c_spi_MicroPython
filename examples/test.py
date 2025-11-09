# BNO08x MicroPython SPI Test
#
# This program set up an SPI connection to the BNO08x sensor
from time import sleep
from utime import sleep_ms

from bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER, \
    BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR
from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# spi0_RX = Pin(16, Pin.IN)  # spi0_RX, RES (MISO) - connected to BNO SO (POCI)
cs = Pin(17, Pin.OUT, value=1)  # cs for SPI
# sck = Pin(18, Pin.OUT, value=0)  # sck for SPI
# spi0_TX = Pin(19, Pin.OUT, value=0)  # spi0_TX (MOSI) - connected to BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # Wakes BNO to enable INT response

spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=1_000_000)

print("Start")
bno = BNO08X_SPI(spi, cs, reset_pin, int_pin, wake_pin, debug=False)
print(spi)  # polarity=1, phase=1 for bno08x
print("====================================\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
print("BNO08x sensors enabled\n")

cpt = 0

while True:
    cpt += 1
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Acceleration  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")


