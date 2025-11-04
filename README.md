# bno08x-i2c-spi-micropython
## Micropython I2C SPi library for 9-axis BNO08X sensors

- 100% inspired by the original Adafruit CircuitPython I2C library for BNO08X
- Copyright (c) 2020 Bryan Siepert for Adafruit Industries
- Also inspired by dobodu

## Library tested

bno08x MicroPython driver for i2c, spi, uart on MicroPython

This library has been tested with BNO086 sensor. It should work with BNO080 and BNO085 sensors. It has been tested with Raspberry Pico 2 W

## Setting up the Driver

### I2C

    #import the library
    import bno08x

    #setup the  I2C bus
    i2c0 = I2C(0, scl=I2C0_SCL, sda=I2C0_SDA, freq=100000, timeout=200000)

    #setup the BNO sensor
    bno = BNO08x(i2c0)

Additional paramters

    bno = BNO08X_I2C(i2c_bus, address=None, rst_pin=14, debug=False)

## i2c Issues with speed and data quality

Unfortunately, The BNO080, BNO085, and BNO086 all use **_non-standard clock stretching_** on the i2c. This can cause a variety of issues including report errors and the need to restart sensor. Clock stretching interferes with various chips (ex: RP2) in different ways. If you see sporadic results this may be part of the issue (BNO08X Datasheet 1000-3927 v1.17, page 15).

## References

The CEVA BNO085 and BNO086 9-axis sensors are made by Ceva (https://www.ceva-ip.com). They are based on Bosch hardware but use Hillcrest Labs’ proprietary sensor fusion software. BNO086 is backwards compatible with BNO085 and both are pin-for-pin replacements for Bosch Sensortec’s discontinued BNO055 and BMF055 sensors.

https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Product-Brief.pdf

https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf

Bosch has a new 6-axis IMU BHI385 (announced June 2025) that can be paired with BMM350 3-axis Geomagnetic sensor.


