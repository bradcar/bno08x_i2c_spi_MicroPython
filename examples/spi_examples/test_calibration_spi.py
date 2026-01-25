# test_calibration_spi.py
#
# BNO08x MicroPython SPI Test
# Calibration of three main sensors.
# see README.md "Basic User Sensor Calibration Procedure" for recommened sensor movements
#
# This code is written with calibration as a function to show how you can use it in other programs.

from bno08x import *
from machine import SPI, Pin
from spi import BNO08X_SPI
from utime import ticks_ms, ticks_diff


def sensor_calibration(bno, stable_seconds):
    """
    Sensor calibration, returns when results arestable for stable_seconds.
    
    :param bno: bno spi object for sensors
    :param stable_seconds: number of seconds required before saves calibration and function returns
    """
    print(f"\nCalibration: Waits for {stable_seconds} secs of Medium(2) to High(3) Accuracy, before saving\n")
    start_good = None
    calibration_good = False
    status = ""

    # Begin calibration, Wait sensor to be ready to calibrate
    bno.begin_calibration()
    bno.calibration_status()

    last_print = ticks_ms()
    while True:
        bno.update_sensors()

        # only print every .2 sec (200 ms)
        if ticks_diff(ticks_ms(), last_print) < 200:
            continue
        last_print = ticks_ms()

        _, _, _, accel_accuracy, _ = bno.acceleration.full
        _, _, _, gyro_accuracy, _ = bno.gyro.full
        _, _, _, mag_accuracy, _ = bno.magnetic.full

        if all(x >= 2 for x in (accel_accuracy,  gyro_accuracy, mag_accuracy)):
            status = "All Sensors >= 2"
            calibration_good = True
        else:
            if start_good is not None:
                print("\nlost calibration, resetting timer\n")
            status = "low accuracy, suggest moving sensor"
            calibration_good = False

        print(f"Accuracy: accel={accel_accuracy}, gyro={gyro_accuracy}, mag={mag_accuracy}\t{status}")

        if calibration_good:
            if start_good is None:
                start_good = ticks_ms()
                print(f"Calibration >=2 on all sensors. Start {stable_seconds}-second timer...\n")
            else:
                elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
                if elapsed >= stable_seconds:
                    print(f"*** Calibration stable for {stable_seconds} secs")
                    break
        else:
            start_good = None

    bno.save_calibration_data()
    print("*** Calibration saved")


def main():
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
    print("====================================\n")

    bno.acceleration.enable(20)
    bno.magnetic.enable(20)
    bno.gyro.enable(20)

    bno.print_report_period()
    
    # function to calibrate sensor
    sensor_calibration(bno, stable_seconds=5.0)


if __name__ == "__main__":
    main()
