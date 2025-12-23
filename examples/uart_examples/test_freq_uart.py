# test_freq_uart.py
#
# BNO08x MicroPython UART Test
#
# Test update report frequency of quaternion.full
# We only print average of report period every 500 iterations
#
# Enabling reports at 400Hz (2.5ms/report), UART can loop at up to 2.7ms.
# At higher frequencies, since controller falls behind the sensor packages multple reports together
# but only returns latest so apparent period appears longer.

from time import sleep

from uart import BNO08X_UART
from bno08x import *

from machine import UART, Pin
from utime import ticks_ms, ticks_us, sleep_us, ticks_diff

# UART1-tx=Pin(8) - BNO SCI
# UART1-rx=Pin(9) - BNO SDA
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset

uart = UART(1, baudrate=3_000_000, tx=Pin(8), rx=Pin(9))

bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin)

print(uart)  # baudrate 3000000 required
print("Start")
print("====================================\n")

bno.quaternion.enable(400)
#bno.acceleration.enable(1000)
bno.print_report_period()

# Every sum_count print average duration
sum_count=500
count = 0
running_sum = 0.0

print("\nStart loop:")

# time of 1st quaternion
bno.update_sensors()
_, _, _, _, _, last_ts_ms = bno.quaternion.full
#x, y, z, _, last_ts_ms = bno.acceleration.full


while True:
    # required to get data from enabled sensors
    while bno.update_sensors() == 0:
        sleep_us(1)
        pass
    
    # Get Quaternion report, in this test, we are not using report values, but could log them to array
    quat_i, quat_j, quat_k, quat_real, _, ts_ms = bno.quaternion.full
    #x, y, z, _, ts_ms = bno.acceleration.full

    running_sum += ts_ms - last_ts_ms
    count +=1
    last_ts_ms = ts_ms
        
    if count % sum_count == 0:
        print(f"ave={running_sum/sum_count:.1f}")
        running_sum=0.0
