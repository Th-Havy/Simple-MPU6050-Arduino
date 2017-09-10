# Simple-MPU6050-Arduino
Simple Arduino library for using the MPU6050 accelerometer &amp; gyroscope sensor from invensense

This library is meant as a starting base when using the MPU6050 sensor. It is meant to be simple and easy to use, as part of an arduino project. In particular, the library does not implement or make use of all the features of the chip (such as the FIFO buffer, Digital Motion Processor), and the code is not optimized for any board. 

The Mpu6050 class represents a sensor (more than one sensor can be used) and can be used to configure it and read measurements.
The technical aspects of understanding how to use the chip are hidden behind a simple interface: the user can read the measurements in real-world units without having to go through all the database to find how to convert a raw 16-bit integer to °/s, etc...

The Mpu6050Data represents the data measured by the sensor: acceleration (Vector3f), gyroscope (Vector3f), temperature (float)

A Vector3f class is used by the two previous classes; it is a simple class to represent a 3D vector with 3 float components. It has some useful methods such as magnitude and dot product, and some operator have been overloaded (+,-,/,...) to make vector computations easier to read. 

Here's a list of some of the supported features:
- Read measurements in physical units (gyro:°/s, accel:m/s^2, temp:°C)
- Gyroscope and accelerometer configuration (selecting the range of measurements)
- Digital Low Pass Filter (DLPF) configuration
- Multiple sensors (up to 2 sensors on the same I2C bus, support of multiple I2C buses)
- Enter and Exit sleep mode (power saving mode)

Here's a list of some features NOT supported (on purpose):
- Interrupts
- FIFO buffer
- Digital Motion Processor (DMP)
