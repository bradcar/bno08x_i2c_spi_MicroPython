# BNO08x MicroPython I2C Test
#
# This program set up an I2C connection to the BNO08x sensor

from time import sleep

from i2c import BNO08X_I2C
from bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ACCELEROMETER

from machine import I2C, Pin
from utime import ticks_ms, sleep_us

int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, BNO (RST) signals when ready
reset_pin = Pin(15, Pin.OUT)  # Reset, tells BNO (INT) to reset

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)

print("Start")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("====================================")

# TODO BRC ... Make sure WAKE_PIN=NONE in i2c.py
bno = BNO08X_I2C(i2c0, address=0x4B, debug=False)
#bno = BNO08X_I2C(i2c0, address=0x4B, reset_pin=reset_pin, int_pin=int_pin, debug=True)

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)

# Enabled Report Periods:
#   bno.enable_feature(BNO_REPORT_ACCELEROMETER, 50)
#   got 62.5 Hz NOT 50 Hz
#   1: ACCELEROMETER, 16.0 ms, 62.5 Hz
# ~16ms 
# Accel  X: -0.996  Y: -0.277  Z: +9.473  m/s² - 9 ms
# Accel  X: -0.996  Y: -0.277  Z: +9.473  m/s² - 3 ms
# Accel  X: -0.996  Y: -0.277  Z: +9.473  m/s² - 2 ms
# Accel  X: -0.996  Y: -0.277  Z: +9.473  m/s² - 2 ms

# Enabled Report Periods:
#   bno.enable_feature(BNO_REPORT_ACCELEROMETER, 100)
#   got 125 Hz NOT 100 Hz
#   1: ACCELEROMETER,  8.0 ms, 125.0 Hz
# ~16ms
# Accel  X: -0.977  Y: -0.289  Z: +9.469  m/s² - 16 ms

# Enabled Report Periods:
#   bno.enable_feature(BNO_REPORT_ACCELEROMETER, 200)
#   got 250 Hz NOT 200 Hz
#   1: ACCELEROMETER,  4.0 ms, 250.0 Hz
# ~50ms !!!!!! need to debug
# Accel  X: -0.977  Y: -0.277  Z: +9.496  m/s² - 55 ms

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
    print(f"Accel  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s² - {end-start} ms")
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
    print()