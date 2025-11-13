# bno08x-i2c-spi-micropython
## Micropython I2C SPi library for 9-axis BNO08X sensors

- 100% inspired by the original Adafruit CircuitPython I2C library for BNO08X
- Copyright (c) 2020 Bryan Siepert for Adafruit Industries
- This code also inspired by feature and fixes written by dobodu

## Library tested

bno08x MicroPython driver for i2c, spi, uart on MicroPython

This library has been tested with BNO086 sensor. It should work with BNO080 and BNO085 sensors. It has been tested with Raspberry Pico 2 W

## Setting up to use the Sensor

### I2C Setup

    # import the library
    from i2c import BNO08X_I2C
    from bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ACCELEROMETER

    # set up the  I2C bus
    i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)

    # set up the BNO sensor on I2C
    bno = BNO08X_I2C(i2c0, address=0x4b)
- 
- address : if using 2 BNO08x each needs a separate address (depending on board, add solder jump or cut wire).

Optional parameters: 

    bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=Pin(12), int_pin=Pin(13), wake_pin=Pin(21), debug=False)

Required by SPI
- reset_pin : Needed by SPI to operate correctly. This is also used to enable sensor hard reset (Pin object, not number). If not defined, a soft reset is used.
- int_pin : Needed by SPI to operate correctly. Also used to synchronize sensor time with host to enable microsecond accuracy timestamps. Define a Pin object. Not required if only 200-millisecond host-based timestamps are adequate, or if timestamps are not required.
- wake_pin : Needed by SPI to operate correctly. 

Optional
- debug : print very detailed logs, mainly for debugging driver.

## Enable the sensor reports

Before getting sensor results the reports must be enabled:

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)  # for accelerometer
    
Primary sensor reports:

        BNO_REPORT_ACCELEROMETER
        BNO_REPORT_GYROSCOPE
        BNO_REPORT_MAGNETOMETER
        BNO_REPORT_LINEAR_ACCELERATION
        BNO_REPORT_ROTATION_VECTOR
        BNO_REPORT_GRAVITY
        BNO_REPORT_GAME_ROTATION_VECTOR
        BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
        BNO_REPORT_STEP_COUNTER
        BNO_REPORT_SHAKE_DETECTOR
        BNO_REPORT_STABILITY_CLASSIFIER
        BNO_REPORT_ACTIVITY_CLASSIFIER

Additional reports:

        BNO_REPORT_RAW_ACCELEROMETER
        BNO_REPORT_RAW_GYROSCOPE
        BNO_REPORT_RAW_MAGNETOMETER
        BNO_REPORT_UNCALIBRATED_GYROSCOPE
        BNO_REPORT_UNCALIBRATED_MAGNETOMETER
        BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR
        BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR
        BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR

There are additional sensor reports that this driver has not fully implemented. See code and references for details.

## Getting the sensor results:

Sensors values can be accessed with:

    accel_x, accel_y, accel_z = bno.acceleration

Roll, tilt, and yaw can be obtained with:

    roll, tilt, yaw = bno.euler

**Examples of other sensor reports**

See examples directory for sample code. The following functions use on-chip sensor fusion for accuracy.

    x, y, z = bno.acceleration  # acceleration 3-tuple of x,y,z float returned
    x, y, z = bno.linear_acceleration  # linear accel 3-tuple of x,y,z float returned
    x, y, z = bno.gyro  # acceleration 3-tuple of x,y,z float returned
    x, y, z = bno.magnetic  # acceleration 3-tuple of x,y,z float returned
    roll, pitch, yaw = bno.euler  # rotation 3-tuple of x,y,z float returned
    x, y, z = bno.gravity  # vector 3-tuple of x,y,z float returned
    i, j, k, real = bno.quaternion  # rotation 4-tuple of i,j,k,real float returned
    i, j, k, real = bno.geomagnetic_quaternion  # rotation 4-tuple of i,j,k,real float returned
    i, j, k, real = bno.game_quaternion  # rotation 4-tuple of i,j,k,real float returned
    num = bno.steps  # number of steps since initialization returned
    state = bno.shake  # boolean of state since last read returned
    stability_str = bno.stability_classification  # string of stability classification returned
    activity_str = bno.activity_classification  # string of activity classification returned

The following functions can be used to tare, calibrate, and test the sensor:

    bno.tare  # tare the sensor

    bno.begin_calibration  # begin calibration
    mag_accuracy = bno.calibration_status  # magnetic calibration status int returned
    print(f"Mag Calibration: {REPORT_ACCURACY_STATUS[mag_accuracy]} = {mag_accuracy}")
    bno.save_calibration_data  # Save calibration

    status = bno.ready  # test sensor status, boolean returned

The following functions provide raw values directly from individual sensors, they lack the advanced on-sensor software that make the above functions more accurate:

    # raw data sensor tuple of x,y,z, float and time_stamp int returned
    x, y, z, usec = bno.raw_acceleration
    x, y, z, usec = bno.raw_magnetic
    
    # raw data gyro tuple of x,y,z, celsius float, and time_stamp int returned
    x, y, z, temp_c, usec = bno.raw_gyro

## Option to Change Sensor Report Frequency

The sensor report default frequency is 20 Hz. The number of reports per second that the BNO08X can reliably deliver is dependent on the interface bandwidth
and the number of reports that a BNO08X is asked to generate. I2C will definitely limit this frequency (est 10 to 50 Hz with a few reports). One should consider SPI for higher frequencies.
Refer to the BNO080_085-Datasheet.pdf (page 50) for Maximum sensor report rates by report type. Some sensor reports can be updated at 400 to 400 Hz on SPI (untested). If your code request faster than the report feature frequency specified, repeated values will be returned.

Before getting sensor results the reports must be enabled:

    bno.enable_feature(BNO_REPORT_ACCELEROMETER, 40)  # enable accelerometer reports at 40 Hertz

## Euler angles, gimbal lock, and quaternions

Euler angles have a problem with Gimbal lock. This is where a loss of a degree of freedom occurs when two rotational axes align, which means certain orientations have multiple representations. There was a famous example of this on Apollo 11.
Quaternions avoid this problem because they represent a rotation as a single axis and an angle, providing a unique representation for every possible orientation. 

- https://base.movella.com/s/article/Understanding-Gimbal-Lock-and-how-to-prevent-it?language=en_US
- https://en.wikipedia.org/wiki/Gimbal_lock

## I2C Issues with speed and data quality

Unfortunately, the BNO080, BNO085, and BNO086 all use **_non-standard clock stretching_** on the I2C. This can cause a variety of issues including report errors and the need to restart sensor. Clock stretching interferes with various chips (ex: RP2) in different ways. If you see sporadic results this may be part of the issue (BNO08X Datasheet 1000-3927 v1.17, page 15).

## SPI Setup

Requirements to using Sparkfun BNO086 with SPI
1. must clear i2c jumper when using SPI or UART (https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/board_files/SparkFun_VR_IMU_Breakout_BNO086_QWIIC_Schematic_v10.pdf)
2. must have solder blob ONLY on SP1, must have Wake pin connect to a pin.
3. UART MUST be set to baudrate=3000000

In order to use SPI on most sensor boards instead of I2C you must often have to add ONE solder blob on PS1. 
On the back side of Sparkfun BNO086 and Adafruit BNO085, one needs a solder blob to bridge PS1.
PS0 must be connected to a GPIO so it can be pulsed low to serve as the SPI's WAKE functionality can be performed.
Ceva specifies that PS0 and PS1 must be high during SPI operation, but PS0 is set low and then high in driver to wake bno08x.

If you put a solder blob on both PS0 and PS1, this driver is likely to hang.

    from machine import SPI, Pin
    from spi import BNO08X_SPI
    from bno08x import BNO_REPORT_ACCELEROMETER

    int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
    reset_pin = Pin(15, Pin.OUT)  # Reset to signal BNO to reset
    cs = Pin(17, Pin.OUT)  # cs for SPI
    wake_pin = Pin(20, Pin.OUT, value=1)  # Wakes BNO to enable INT response


    spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=3_000_000)
    print(spi)

    bno = BNO08X_SPI(spi, cs, int_pin, reset_pin, wake_pin, debug=False)

## UART Setup

PS0 and PS1 are the host interface protocol selection pins, therefore UART can not use wake pin.  In order to use UART, PS1 must be high (solder blob) and PS0/WAKE not have solder blob so it is tied to ground.

1. must clear i2c jumper when using SPI or UART (https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/board_files/SparkFun_VR_IMU_Breakout_BNO086_QWIIC_Schematic_v10.pdf)
2. must have solder blob ONLY on SP1, must NOT have Wake pin connect to a pin.

## UART-RVC - NOT SUPPORTED (RVC, Robot Vacuum Cleaners)

The BNO08X has a simplified UART interface for use on unmanned ground roving robot and robot vacuum cleaners (RVC).
This is a very different protocol and not supported in my driver. Take a look at: https://github.com/rdagger/micropython-bno08x-rvc

## Report Maximum Frequencioes

| **Feature**             | **Max Frequency (Hz)** | **msec/Report** |
|-------------------------|------------------------|-----------------|
| Composite Gyro Rotation | 1000                   | 1.0 ms          |
| Accelerometer           | 500                    | 2.0 ms          |
| Rotation Vector         | 400                    | 2.5 ms          |
| Gaming Rotation         | 400                    | 2.5 ms          |
| Gravity                 | 400                    | 2.5 ms          |
| Linear Acceleration     | 400                    | 2.5 ms          |
| Gyroscope               | 400                    | 2.5 ms          |
| Magnetometer            | 100                    | 10.0 ms         |
| Geomagnetic Rotation    | 90                     | 11.1 ms         |
| (report default)        | 20                     | 50.0 ms         |

Report frequencies should be enabled before requesting reports

    bno.enable_feature(BNO_REPORT_ACCELEROMETER, 100)  # Enable accelerometer reports at 100 Hertz

When the frequency of the sensor is set in enable_feature, it should be viewed as a suggestion to operate at that interval.
If the sensor cannot operate as requested, it may operate faster or slower (SH-2 datasheet 5.4.1 Rate Control).

On can access the sensors report period for each with this function. Each sensor may be faster or slower in a single program.

    accelerometer_period_us = bno.report_period_us(BNO_REPORT_ACCELEROMETER)
    period_ms = accelerometer_period_us / 1000.0
    print(f"Accelerometer: {period_ms:.1f} ms, {1_000 / period_ms:.1f} Hz")

With a single feature, we've seen the above requested 100 Hz have the sensor report at 125Hz. With multiple featues we've also seen 20Hz changed to 10 Hz.

## Calibration

background: https://www.youtube.com/watch?v=0rlvvYgmTvI&t=28s

## References

The CEVA BNO085 and BNO086 9-axis sensors are made by Ceva (https://www.ceva-ip.com). They are based on Bosch hardware but use Hillcrest Labs’ proprietary sensor fusion software. BNO086 is backwards compatible with BNO085 and both are pin-for-pin replacements for Bosch Sensortec’s discontinued BNO055 and BMF055 sensors.

- https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Product-Brief.pdf

- https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

- https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf

Bosch has a new 6-axis IMU BHI385 (announced June 2025) that can be paired with BMM350 3-axis Geomagnetic sensor.
