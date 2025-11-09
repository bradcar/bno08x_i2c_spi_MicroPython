# BNO08x MicroPython SPI Test
#
# This program set up an SPI connection to the BNO08x sensor
from time import sleep
from utime import sleep_ms

from bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER, \
    BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR, REPORT_ACCURACY_STATUS
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

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, 10)
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 10)
bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_MAGNETOMETER, 20)
bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_GYROSCOPE, 20)
print("BNO08x sensors enabled\n")

calibration_status = bno.calibration_status
print(f"Mag Calibration: \"{REPORT_ACCURACY_STATUS[calibration_status]}\" = {calibration_status}")


cpt = 0

while True:
    sleep(0.1)
    cpt += 1

    print("{cpt=}")
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Acceleration  X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")
    gyro_x, gyro_y, gyro_z = bno.gyro
    print(f"Gyroscope     X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")
    mag_x, mag_y, mag_z = bno.magnetic
    print(f"Magnetometer  X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"Rot Vect Quat I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")
    roll, tilt, yaw = bno.euler
    print(f"Euler Angle   R: {roll:+.3f}  T: {tilt:+.3f}  Y: {yaw:+.3f}")
    print()
