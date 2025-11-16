# BNO08x MicroPython UART Test
#
# This program set up an UART connection to the BNO08x sensor

from time import sleep

# from backup_lib.uart import BNO08X_UART
# from backup_lib.bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER, \
#     BNO_REPORT_ACCELEROMETER
from uart import BNO08X_UART
from bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER, \
    BNO_REPORT_ACCELEROMETER

from machine import UART, Pin
from utime import ticks_ms, sleep_us

# TXO — Output pin for serial UART transmit pin. Data is sent out to the microcontroller to this pin.
#       Also connected to the I2C's SDA and SPI's POCI pin.
# RXI — Input pin for the serial UART receive pin. Data is received from the microcontroller to this pin.
#       Also connected to the I2C's SCL and SPI's SCK pin.
# txo = Pin(12)
# rxi = Pin(13)
int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT, value=1)  # Reset, tells BNO (INT) to reset
# wake_pin can NOT be used, wake/PS0 must be ground

uart = UART(1, baudrate=3_000_000, tx=Pin(8), rx=Pin(9), timeout=2000)

print("Start")
print("====================================")

#bno = BNO08X_UART(uart, debug=False)
#bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin, debug=True)
bno = BNO08X_UART(uart, reset_pin=reset_pin, int_pin=int_pin, debug=True)

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 125)

# bno.enable_feature(BNO_REPORT_MAGNETOMETER, 20)
# bno.enable_feature(BNO_REPORT_GYROSCOPE, 20)
# bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 10)
# bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

print("BNO08x sensors enabled\n")
bno.print_report_period()

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

start = ticks_ms()
while True:
    cpt += 1

    accel_x, accel_y, accel_z = bno.acceleration
    end = ticks_ms()
    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s² - {end - start} ms")
    start = end

#     print(f"Acceleration  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")
#     gyro_x, gyro_y, gyro_z = bno.gyro
#     print(f"Gyroscope     X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")
#     mag_x, mag_y, mag_z = bno.magnetic
#     print(f"Magnetometer  X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT")
#     quat_i, quat_j, quat_k, quat_real = bno.quaternion
#     print(f"Rot Vect Quat I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
#     roll, tilt, yaw = bno.euler
#     print(f"Euler Angle   R: {roll:+.3f}  T: {tilt:+.3f}  Y: {yaw:+.3f}")
#     print()