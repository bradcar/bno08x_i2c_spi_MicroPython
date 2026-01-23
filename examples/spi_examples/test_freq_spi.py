# test_freq_spi.py
#
# BNO08x MicroPython SPI Test
#
# Test update report frequency of quaternion.full
# We print the average of report period every 500 iterations
#
# Enabling reports at 400Hz (2.5ms/report), SPI can loop at up to 2.5ms.
# At higher frequencies, since controller falls behind the sensor packages multple reports together
# but only returns latest so apparent period appears longer.
#
# Fastest results on Raspberry Pi Pico 2W and BNO086
#
#               Request  Actual  Period
# Quaternion     400 Hz  400 Hz  2.5 ms  BNO will not enable higher than 500 Hz
# Quaternion     500 Hz  500 Hz  5.3 ms  Likely multiple reports in same update, this driver only reports latest
# Acceleration   400 Hz  500 Hz  2.0 ms  BNO will not enable higher than 500 Hz

import gc

from time import sleep
from utime import sleep_ms, ticks_ms

from bno08x import *

from machine import SPI, Pin
from spi import BNO08X_SPI

int_pin = Pin(14, Pin.IN)  # Interrupt, enables BNO to signal when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset to signal BNO to reset

# miso=Pin(16) - BNO SO (POCI)
cs_pin = Pin(17, Pin.OUT, value=1)
# sck=Pin(18)  - BNO SCK 
# mosi=Pin(19) - BNO SI (PICO)
wake_pin = Pin(20, Pin.OUT, value=1)  # BNO WAK

spi = SPI(0, baudrate=3000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

bno = BNO08X_SPI(spi, cs_pin, reset_pin, int_pin, wake_pin)

print(spi)
print("Start")
print("====================================\n")

bno.quaternion.enable(400)
# bno.acceleration.enable(500)
bno.print_report_period()

# Every sum_count print average duration
sum_count=500
count = 0
running_sum = 0.0

print("\nStart loop:")

# time of 1st quaternion
bno.update_sensors()
_, _, _, _, _, last_ts_ms = bno.quaternion.full
# _, _, _, _, last_ts_ms = bno.acceleration.full

gc.collect()

quat = bno.quaternion
# accel = bno.acceleration

while True:
    # required to get data from enabled sensors
    while bno.update_sensors() == 0:
        pass
    
    # Get Quaternion report, in this test, we are not using report values, but could log them to array
    qr, qi, qj, qk, _, ts_ms = quat.full
    # x, y, z, _, ts_ms = accel.full

    running_sum += ts_ms - last_ts_ms
    count +=1
    last_ts_ms = ts_ms
        
    if count % sum_count == 0:
        print(f"ave={running_sum/sum_count:.1f}")
        running_sum=0.0
        gc.collect()
