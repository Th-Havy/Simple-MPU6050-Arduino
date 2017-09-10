/* *****************************************************************************
 * Filename: Mpu6050.h
 *
 * Description: This is file contains the definition of the Mpu6050 class, that
 * is used to configure and read data from the eponymous sensor.
 * Each instance of this class represents a different sensor that has an I2C
 * address 0x68 or 0x69 if the AD0 pin is connected to logic 1 (3.3V).
 * The Mpu6050Data struct is also defined here and has 3 member variables for
 * each type of measurement (accel,gyro,temp).
 * A set of useful enumerations is defined for configuring the sensor in a more
 * readable way.
 *
 * Author: Thomas Havy
 *
 * License: MIT License - Copyright (c) 2017 Thomas Havy
 * A copy of the license can be found at:
 * https://github.com/Th-Havy/Simple-MPU6050-Arduino/blob/master/LICENSE
 *
 * Changes:
 * - Created: 07-Sept-2017
 *
 * ************************************************************************* */

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "Vector3f.h"

struct Mpu6050Data
{
    public:
        Vector3f acceleration; // m/s^2
        Vector3f gyroscope; // degree/s
        float temperature; // Celsius degrees
};

// Used to select the gyroscope measurement range.
// Always select the smallest possible range for precision.
enum Mpu6050GyroscopeRange
{
    Max250Dps, // +/- 250 degree per second of rotational speed
    Max500Dps,
    Max1000Dps,
    Max2000Dps
};

// Used to select the accelerometer measurement range.
// Always select the smallest possible range for precision.
enum Mpu6050AccelerometerRange
{
    Max2g, // +/- 2g (19.6 m/s^2) of acceleration
    Max4g,
    Max8g,
    Max16g
};

// Used to configure the internal Digital Low Pass Filter (DLPF)
// The DLPF will average the measurement which is useful for slow varying
// motion, as it removes high frequency components (such as vibrations)
enum Mpu6050DLPFBandwidth
{
    Max260Hz, // 260 measurements per second
    Max184Hz,
    Max94Hz,
    Max44Hz,
    Max21Hz,
    Max10Hz,
    Max5Hz
};

class Mpu6050
{
    public:
        // Specify AD0 pin level and I2C bus when instantiating a device
        Mpu6050(bool ad0 = LOW, TwoWire *bus = &Wire);

        // Initialize the device before reading data
        bool init(Mpu6050AccelerometerRange accelRange = Max2g,
                  Mpu6050GyroscopeRange gyroRange = Max250Dps,
                  Mpu6050DLPFBandwidth bandwidth = Max260Hz,
                  byte SampleRateDivider = 7);

        // Methods to read measurements
        Mpu6050Data readData();
        Vector3f readAcceleration();
        Vector3f readGyroscope();
        float readTemperature();

        // Configuration of the sensors
        void setAccelerometerRange(Mpu6050AccelerometerRange range);
        void setGyroscopeRange(Mpu6050GyroscopeRange range);
        void setDLPFBandwidth(Mpu6050DLPFBandwidth bandwidth);
        void setSampleRateDivider(byte divider); // sample rate = 8kHz / (1 + divider)

        Mpu6050AccelerometerRange getAccelerometerRange();
        Mpu6050GyroscopeRange getGyroscopeRange();
        Mpu6050DLPFBandwidth getDLPFBandwidth();
        byte getSampleRateDivider();

        // Resets the registers to their default value (init() should be called
        // after a reset to use again the device)
        void reset();

        // Return true if the connection is established with the MPU6050
        bool isConnected();

        // Set sensor in sleep mode (power saving mode, no data measurements)
        void sleepMode();
        void wakeUp();

        void set_ad0(bool ad0);
        bool get_ad0();

        // Change the I2C bus (there is only one on the Arduino uno)
        void setBus(TwoWire* bus);
        TwoWire* getBus();

    private:
        byte m_address;
        Mpu6050AccelerometerRange m_accelerometerRange;
        Mpu6050GyroscopeRange m_gyroscopeRange;

        // Allow the use of devices on different I2C buses
        TwoWire *m_wire;

        // Raw data to real-world units
        float rawTemperatureToCelsius(int16_t rawTemperature);
        float rawGyroscopeToDps(int16_t rawGyroscope); // degree/s
        float rawAccelerationToMps2(int16_t rawAcceleration); // m/s^2

        // For convenience:
        byte Mpu6050::read8(byte registerAddr);
        int16_t Mpu6050::read16(byte registerAddr);
        byte Mpu6050::write8(byte registerAddr, byte value);
};

#endif // MPU6050_H
