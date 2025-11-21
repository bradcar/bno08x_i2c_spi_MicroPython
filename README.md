# bno08x-i2c-spi-micropython
## Micropython I2C SPi library for 9-axis BNO08X sensors

- 100% inspired by the original Adafruit CircuitPython I2C library for BNO08X
- Copyright (c) 2020 Bryan Siepert for Adafruit Industries
- This code also inspired by feature and fixes written by dobodu

## Library tested

bno08x MicroPython driver for I2C, SPI, UART on MicroPython

This library has been tested with BNO086 sensor. It should work with BNO080 and BNO085 sensors. 
It has been tested with Raspberry Pico 2 W.

This driver is written to provide a simple interface and results for slow updates. It also provides 0.1 msec resolution 
timestamps for each sensor report, because knowing IMU results together with timestamp of results is critical for many
telemetry applications.  This driver is written to be efficient because these sensors can update results as frequently
as every 4ms to 10ms (depending on the report requested).

## Setting up to use the Sensor

### I2C Setup

    # import the library
    from i2c import BNO08X_I2C
    from bno08x import *

    # set up the  I2C bus
    int_pin = Pin(14, Pin.IN, Pin.PULL_UP)
    i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)

    # set up the BNO sensor on I2C
    bno = BNO08X_I2C(i2c0, address=0x4b, int_pin=int_pin)

Required for I2C (see SPI and UART below):
- address : each BNO08x needs a separate address (depending on board, add solder jump or cut wire).
- int_pin : required for accurate sensor timestamps. Define a Pin object, not  number.

Optional parameters:

    bno = BNO08X_I2C(i2c0, address=0x4b, reset_pin=Pin(12), int_pin=Pin(13), debug=False)

Optional for I2C:
- reset_pin : used by I2C for hardware reset, if not defined uses soft reset. It is a Pin object, not number
- debug : print very detailed logs, mainly for debugging driver.

PS0 (Wake_pin) and PS1 are used to select I2C, therefore I2C can not use wake pin.
In order to use I2C, PS1 can not have solder blob so it is tied to ground and PS0 (Wake_pin) can not have solder blob so it is tied to ground.

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

## Getting the sensor results:

Sensors values can be accessed with:

    accel_x, accel_y, accel_z = bno.acceleration

Roll, tilt, and yaw can be obtained with:

    roll, tilt, yaw = bno.euler

The data and the metadata for each report can be accessed at the same time using ".full".
In this way, the accuracy and the usec-accurate timestamp  of a particular report is returned at the same time.
The timestamp_us is synchronized with the host's usec time. Understanding timestamps is recommended for
high-frequency applications (>5Hz).

    accel_x, accel_y, accel_z, accuracy, timestamp_us = bno.acceleration.full

Metadata on the accuracy and the usec-accurate timestamp can also be separately accessed. However, unless carefully
designed in the user code, they may be from a different report.

    accuracy, timestamp_us = bno.acceleration.meta

**Examples of other sensor reports**

The examples directory shows the use of the following sensor reports. Each of these functions use on-chip sensor fusion for accuracy.

    x, y, z = bno.acceleration          # acceleration 3-tuple of x,y,z float returned
    x, y, z = bno.linear_acceleration   # linear accel 3-tuple of x,y,z float returned
    x, y, z = bno.gyro                  # acceleration 3-tuple of x,y,z float returned
    x, y, z = bno.magnetic              # acceleration 3-tuple of x,y,z float returned
    roll, pitch, yaw = bno.euler        # rotation 3-tuple of x,y,z float returned
    x, y, z = bno.gravity               # vector 3-tuple of x,y,z float returned
    i, j, k, real = bno.quaternion      # rotation 4-tuple of i,j,k,real float returned
    i, j, k, real = bno.geomagnetic_quaternion  # rotation 4-tuple of i,j,k,real float returned
    i, j, k, real = bno.game_quaternion         # rotation 4-tuple of i,j,k,real float returned
    num = bno.steps                     # number of steps since sensor initialization returned
    state = bno.shake                   # boolean of state since last read returned
    stability_str = bno.stability_classification    # string of stability classification returned
    activity_str = bno.activity_classification      # string of activity classification returned

The following functions can be used to tare, calibrate, and test the sensor:

    bno.tare  # tare the sensor

    bno.begin_calibration  # begin calibration
    mag_accuracy = bno.calibration_status  # magnetic calibration status int returned
    print(f"Mag Calibration: {REPORT_ACCURACY_STATUS[mag_accuracy]} = {mag_accuracy}")
    bno.save_calibration_data  # Save calibration

    status = bno.ready  # test sensor status, boolean returned

## Option to Change Sensor Report Frequency

The sensor report default frequencies are 10 to 20 Hz. The number of reports per second that the BNO08X can reliably
deliver is dependent on the interface bandwidth and the number of reports that a BNO08X is asked to generate. Currently,
on a Raspberry Pi Pico 2 W we can support reports with frequencies up to 125 Hz (8ms).

I2C will definitely limit this frequency (est 10 to 50 Hz with a few reports). One should consider SPI for higher frequencies.
Refer to the BNO080_085-Datasheet.pdf (page 50) for Maximum sensor report rates by report type.
Some sensor reports can be updated at 400 to 400 Hz on SPI (untested). If your code request faster than the report
feature frequency specified, repeated values will be returned.

    bno.enable_feature(BNO_REPORT_ACCELEROMETER, 40)  # enable accelerometer reports at 40 Hertz

## Euler angles, gimbal lock, and quaternions

Euler angles have a problem with Gimbal Lock. With Euler angles, a loss of a degree of freedom occurs when two
rotational axes align, which means certain orientations have multiple representations. 
There was a famous example of this on Apollo 11.
Quaternions avoid this by providing a unique representation for every possible orientation problem. Theu use several rotation around a single axis and an angle.

- https://base.movella.com/s/article/Understanding-Gimbal-Lock-and-how-to-prevent-it?language=en_US
- https://en.wikipedia.org/wiki/Gimbal_lock

## I2C Issues with speed and data quality

Unfortunately, the BNO080, BNO085, and BNO086 all use **_non-standard clock stretching_** on the I2C. This can cause a variety of issues including report errors and the need to restart sensor. Clock stretching interferes with various chips (ex: RP2) in different ways. If you see sporadic results this may be part of the issue (BNO08X Datasheet 1000-3927 v1.17, page 15).

## SPI Setup - for higher speed sensor reports

Requirements to using Sparkfun BNO086 with SPI
1. One must clear i2c jumper when using SPI or UART (https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/board_files/SparkFun_VR_IMU_Breakout_BNO086_QWIIC_Schematic_v10.pdf)
2. One must have solder blob ONLY on SP1, must have Wake pin connect to a pin.
3. UART must be set to baudrate=3000000

In order to use SPI on most sensor boards instead of I2C you must often have to add ONE solder blob on PS1. 
On the back side of Sparkfun BNO086 and Adafruit BNO085, one needs a solder blob to bridge PS1 which will set PS1 high for SPI operation. 
The PS0 (Wake_pin) must be connected to a gpio (wake_pin), be careful not put a solder blog on PS0.
This driver uses the wake-pin after reset as a ‘wake’ signal taking the BNO08X out of sleep for communication with the BNO08X.

    from machine import SPI, Pin
    from spi import BNO08X_SPI
    from bno08x import *

    int_pin = Pin(14, Pin.IN, Pin.PULL_UP)  # Interrupt, enables BNO to signal when ready
    reset_pin = Pin(15, Pin.OUT)  # Reset to signal BNO to reset
    cs_pin = Pin(17, Pin.OUT)  # cs for SPI
    wake_pin = Pin(20, Pin.OUT, value=1)  # Wakes BNO to enable INT response

    spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16), baudrate=3_000_000)
    print(spi)

    bno = BNO08X_SPI(spi, cs_pin=cs_pin, int_pin=int_pin, reset_pin=reset_pin, wake_pin=wake_pin, debug=False)

Required for SPI
- int_pin : Required by SPI for accurate sensor timestamps. Define a Pin object, not  number.
- wake_pin : Required by SPI to operate correctly.
Optional for SPI
- reset_pin : used by SPI for hardware reset, if not defined uses soft reset. It is a Pin object, not number

## UART Setup
UART wires are in some sense opposite of i2c wires (double-check your wiring).
uart = UART(1, baudrate=3_000_000, tx=Pin(8), rx=Pin(9), timeout=2000)
uart = UART(0, baudrate=3_000_000, tx=Pin(12), rx=Pin(13), timeout=2000)

Required for UART:
- int_pin : required by UART for accurate sensor timestamps. Define a Pin object, not number.
Optional for UART:
- reset_pin : used by UART for hardware reset, if not defined uses soft reset. It is a Pin object, not number

1. BNO08x SDA to board UARTx-RX (uart 1 ex: gpio9, uart 0 ex: gpio13)
2. BNO08x SCL to board UARTx-TX (uart 1 ex: gbio8, uart 0 ex: gpio12)
3. todo ? INT Pin is required for accurate communication

PS0 and PS1 are the host interface protocol selection pins, therefore UART can not use a wake pin.  In order to use UART, PS1 must be high (solder blob) and PS0/WAKE not have solder blob so it is tied to ground.

1. must clear i2c jumper when using SPI or UART (https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/board_files/SparkFun_VR_IMU_Breakout_BNO086_QWIIC_Schematic_v10.pdf)
2. must have solder blob ONLY on SP1, must NOT have Wake pin connect to a pin.

## Report Maximum Frequencioes

Currently, this driver supports a maximum report frequency of about 125 Hz (8ms period).

| **Feature**             | **Max Frequency (Hz)** | **msec/Report** | **period we've seen**  |
|-------------------------|------------------------|-----------------|------------------------|
| Composite Gyro Rotation | 1000? (only 500)       | 1.0 ms          | 1 ms                   |
| Accelerometer           | 500                    | 2.0 ms          | 2, 4, 8, 16, 32, 64... |
| Rotation Vector         | 400                    | 2.5 ms          |                        |
| Gaming Rotation         | 400                    | 2.5 ms          | 5, 10, 20, 40, 50, 60  |
| Gravity                 | 400                    | 2.5 ms          |                        |
| Linear Acceleration     | 400                    | 2.5 ms          |                        |
| Gyroscope               | 400                    | 2.5 ms          | 5, 10, 20, 40, 50, 60  |
| Magnetometer            | 100                    | 10.0 ms         | 10, 20, 40, 50, 60,    |
| Geomagnetic Rotation    | 90                     | 11.1 ms         |                        |
| raw Gyroscope           |                        |                 | 10, 20, 40, 50, 60     |
| raw Magnetometer        |                        |                 | 50, 60,                |
| raw Accelerometer       |                        |                 | 32, 64, 96             |
| (report default)        | 20                     | 50.0 ms         |                        |

Report frequencies should be enabled before requesting reports. To convert from period in ms to Hz (1000000/period.)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER, 100)  # Enable accelerometer reports at 100 Hertz

When the frequency of the sensor is set in enable_feature, it should be viewed as a suggestion to operate at that interval.
If the sensor cannot operate as requested, it may operate faster or slower (SH-2 datasheet 5.4.1 Rate Control).

On can access the sensors report period for each with this function. Each sensor may be faster or slower in a single program.

    accelerometer_period_us = bno.report_period_us(BNO_REPORT_ACCELEROMETER)
    period_ms = accelerometer_period_us / 1000.0
    print(f"Accelerometer: {period_ms:.1f} ms, {1_000 / period_ms:.1f} Hz")

With a single feature, we've seen the above requested 100 Hz have the sensor report at 125Hz. With multiple featues we've also seen 20Hz changed to 10 Hz.

## Calibration of the Sensor

background: https://www.youtube.com/watch?v=0rlvvYgmTvI&t=28s

## Raw Reports - Be Careful
    BNO_REPORT_RAW_ACCELEROMETER
    BNO_REPORT_RAW_GYROSCOPE
    BNO_REPORT_RAW_MAGNETOMETER

The raw reports, which do not use sensor fusion calculations, are also implemented in this driver for acceleration,
magnetic, and gyro sensors. It is not generally recommended to use these reports, and they require signficant math (careful
calibaration, Kalman filters, etc.). Please read all references below when attempting to use raw reports.
In addition, there are other sensor reports possible with the bno08x sensors that this driver has not fully
implemented. See code and references for details. The timestamps are not well-documented in Ceva documentation.

## UART-RVC is NOT SUPPORTED by this driver (RVC, Robot Vacuum Cleaners)

The BNO08X has a simplified UART interface for use on unmanned ground roving robot and robot vacuum cleaners (RVC).
This is a very different protocol and not supported in my driver. Take a look at: https://github.com/rdagger/micropython-bno08x-rvc

## References

The CEVA BNO085 and BNO086 9-axis sensors are made by Ceva (https://www.ceva-ip.com). They are based on Bosch hardware but use Hillcrest Labs’ proprietary sensor fusion software. BNO086 is backwards compatible with BNO085 and both are pin-for-pin replacements for Bosch Sensortec’s discontinued BNO055 and BMF055 sensors.

- https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Product-Brief.pdf

- https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

- https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf
- 
-https://cdn.sparkfun.com/assets/7/6/9/3/c/Sensor-Hub-Transport-Protocol-v1.7.pdf

Bosch has a new 6-axis IMU BHI385 (announced June 2025) that can be paired with BMM350 3-axis Geomagnetic sensor.

## Possible Future work

In order to get higher frequency reports it may be necessary to implement this driver with two cores. My thought is the
primary core (core 0) would be running the user program and the second core (core1) would be handling the report
processing and the communications with the bno08x.
