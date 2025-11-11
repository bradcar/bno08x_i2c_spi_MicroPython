# magnetic_calibration_spi.py
#
# BNO08x Micropython magnetic SPI example program

from bno08x import *
from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms
from time import sleep

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

bno.enable_feature(BNO_REPORT_MAGNETOMETER, 10)
print("BNO08x reports enabled\n")
bno.print_report_period()

print("\nBegin calibration: continue until 5 secs of constant \"Medium Accuracy\" to \"High Accuracy\"")
bno.begin_calibration()
calibration_good_at = None
save_pending = False

while True:
    sleep(0.2)
    mag_x, mag_y, mag_z = bno.magnetic
    print(f"\nMagnetometer:  X: {mag_x:.4f}  Y: {mag_y:.4f}  Z: {mag_z:.4f} uT")

    # print accuracy value and string
    calibration_status = bno.calibration_status
    print(f"Magnetic Calibration: \"{REPORT_ACCURACY_STATUS[calibration_status]}\" = {calibration_status}")
    now = ticks_ms()

    # Check when calibration becomes "good"
    if not calibration_good_at and calibration_status >= 2:
        calibration_good_at = now
        print(f"Calibration good at: {now / 1000.0:.3f}s")

    # If calibration stays good for 5 seconds, trigger save
    if calibration_good_at and ticks_diff(now, calibration_good_at) > 5000:
        if not save_pending:
            print("\n*** Calibration stable for 5 sec, saving calibration")
            bno.save_calibration_data()
            save_pending = True
        break

    # Reset timer if calibration drops
    if calibration_status < 2:
        if calibration_good_at:
            print("--- Calibration lost, restarting timer...")
        calibration_good_at = None
        save_pending = False

print("calibration done")