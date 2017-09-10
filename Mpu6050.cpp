/* *****************************************************************************
 * Filename: Mpu6050.cpp
 *
 * Description: This is file contains the implementation of the Mpu6050 class.
 * All the information you need to understand how to communicate and setup the
 * chip can be found in the datasheet and register map from Invensense:
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
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

#include "Mpu6050.h"

// Selected Mpu6050 register addresses (found in the invensense datasheet)
#define MPU6050_REGISTER_SMPRT_DIV 0x19 // Sample rate divider
#define MPU6050_REGISTER_CONFIG 0x1A // DLPF config
#define MPU6050_REGISTER_GYRO_CONFIG 0x1B
#define MPU6050_REGISTER_ACCEL_CONFIG 0x1C
#define MPU6050_REGISTER_ACCEL_XOUT_H 0x3B // Accelerometer measurement
#define MPU6050_REGISTER_ACCEL_XOUT_L 0x3C
#define MPU6050_REGISTER_ACCEL_YOUT_H 0x3D
#define MPU6050_REGISTER_ACCEL_YOUT_L 0x3E
#define MPU6050_REGISTER_ACCEL_ZOUT_H 0x3F
#define MPU6050_REGISTER_ACCEL_ZOUT_L 0x40
#define MPU6050_REGISTER_TEMP_OUT_H 0x41 // Temperature measurement
#define MPU6050_REGISTER_TEMP_OUT_L 0x42
#define MPU6050_REGISTER_GYRO_XOUT_H 0x43 // Gyroscope measurement
#define MPU6050_REGISTER_GYRO_XOUT_L 0x44
#define MPU6050_REGISTER_GYRO_YOUT_H 0x45
#define MPU6050_REGISTER_GYRO_YOUT_L 0x46
#define MPU6050_REGISTER_GYRO_ZOUT_H 0x47
#define MPU6050_REGISTER_GYRO_ZOUT_L 0x48
#define MPU6050_REGISTER_PWR_MGMT_1 0x6B // Power management
#define MPU6050_REGISTER_WHO_AM_I 0x75 // Contains address of the device (0x68)

// Default I2C address of the MPU6050 (0x69 if AD0 pin set to HIGH)
#define MPU6050_DEFAULT_ADDRESS 0x68

// Constant to convert raw temperature to Celsius degrees
#define MPU6050_TEMP_LINEAR_COEF (1.0/340.00)
#define MPU6050_TEMP_OFFSET       36.53

// Constant to convert raw gyroscope to degree/s
#define MPU6050_GYRO_FACTOR_250 (1.0/131.0)
#define MPU6050_GYRO_FACTOR_500  (1.0/65.5)
#define MPU6050_GYRO_FACTOR_1000 (1.0/32.8)
#define MPU6050_GYRO_FACTOR_2000 (1.0/16.4)

// Constant to convert raw acceleration to m/s^2
#define GRAVITATIONAL_CONSTANT_G 9.81
#define MPU6050_ACCEL_FACTOR_2 (GRAVITATIONAL_CONSTANT_G / 16384.0)
#define MPU6050_ACCEL_FACTOR_4 (GRAVITATIONAL_CONSTANT_G / 8192.0)
#define MPU6050_ACCEL_FACTOR_8 (GRAVITATIONAL_CONSTANT_G / 4096.0)
#define MPU6050_ACCEL_FACTOR_16 (GRAVITATIONAL_CONSTANT_G / 2048.0)

Mpu6050::Mpu6050(bool ad0 = LOW, TwoWire *bus = &Wire) :
m_address(MPU6050_DEFAULT_ADDRESS + ad0),
m_wire(bus),
m_accelerometerRange(Max2g),
m_gyroscopeRange(Max250Dps)
{

}

bool Mpu6050::init(Mpu6050AccelerometerRange accelRange = Max2g,
                   Mpu6050GyroscopeRange gyroRange = Max250Dps,
                   Mpu6050DLPFBandwidth bandwidth = Max260Hz,
                   byte SampleRateDivider = 7)
{
    // Pull sensor out of sleep mode
    wakeUp();

    // Test connection between the Arduino and the sensor
    if (!isConnected())
    {
        return false;
    }

    // Reduce output rate of the sensor
    setSampleRateDivider(SampleRateDivider);

    setAccelerometerRange(accelRange);
    setGyroscopeRange(gyroRange);
    setDLPFBandwidth(bandwidth);

    return true;
}

Mpu6050Data Mpu6050::readData()
{
    Mpu6050Data data;

    data.acceleration = readAcceleration();
    data.gyroscope = readGyroscope();
    data.temperature = readTemperature();

    return data;
}

Vector3f Mpu6050::readAcceleration()
{
    int16_t rawAccelX, rawAccelY, rawAccelZ;

    // Read 6 consecutive bytes (2 bytes per axis)
    m_wire->beginTransmission(m_address);
    m_wire->write(MPU6050_REGISTER_ACCEL_XOUT_H);
    m_wire->endTransmission(false);
    m_wire->requestFrom(m_address,6,true);

    // Each accel component is composed of 2 concatenated bytes
    rawAccelX = m_wire->read()<<8 | m_wire->read();
    rawAccelY = m_wire->read()<<8 | m_wire->read();
    rawAccelZ = m_wire->read()<<8 | m_wire->read();

    // Convert each integer value to physical units
    Vector3f accel = Vector3f();
    accel.x = rawAccelerationToMps2(rawAccelX);
    accel.y = rawAccelerationToMps2(rawAccelY);
    accel.z = rawAccelerationToMps2(rawAccelZ);

    return accel;
}

Vector3f Mpu6050::readGyroscope()
{
    int16_t rawGyroX, rawGyroY, rawGyroZ;

    // Read 6 consecutive bytes (2 bytes per axis)
    m_wire->beginTransmission(m_address);
    m_wire->write(MPU6050_REGISTER_GYRO_XOUT_H);
    m_wire->endTransmission(false);
    m_wire->requestFrom(m_address,6,true);

    // Each gyro component is composed of 2 concatenated bytes
    rawGyroX = m_wire->read()<<8 | m_wire->read();
    rawGyroY = m_wire->read()<<8 | m_wire->read();
    rawGyroZ = m_wire->read()<<8 | m_wire->read();

    // Convert each integer value to physical units
    Vector3f gyro = Vector3f();
    gyro.x = rawGyroscopeToDps(rawGyroX);
    gyro.y = rawGyroscopeToDps(rawGyroY);
    gyro.z = rawGyroscopeToDps(rawGyroZ);

    return gyro;
}

float Mpu6050::readTemperature()
{
    int16_t rawTemperature = read16(MPU6050_REGISTER_TEMP_OUT_H);

    return rawTemperatureToCelsius(rawTemperature);
}

void Mpu6050::setAccelerometerRange(Mpu6050AccelerometerRange range)
{
    byte accelRange = static_cast<int>(range) << 3;

    byte accelRangeRegister = read8(MPU6050_REGISTER_ACCEL_CONFIG);

    // Change only the range in the register
    accelRangeRegister = (accelRangeRegister & 0b11100111) | accelRange;

    write8(MPU6050_REGISTER_ACCEL_CONFIG, accelRangeRegister);

    m_accelerometerRange = range;
}

void Mpu6050::setGyroscopeRange(Mpu6050GyroscopeRange range)
{
    byte gyroRange = static_cast<int>(range) << 3;

    byte gyroRangeRegister = read8(MPU6050_REGISTER_GYRO_CONFIG);

    // Change only the range in the register
    gyroRangeRegister = (gyroRangeRegister & 0b11100111) | gyroRange;

    write8(MPU6050_REGISTER_GYRO_CONFIG, gyroRangeRegister);

    m_gyroscopeRange = range;
}

void Mpu6050::setDLPFBandwidth(Mpu6050DLPFBandwidth bandwidth)
{
    byte band = static_cast<byte>(bandwidth);

    byte registerDLPF = read8(MPU6050_REGISTER_CONFIG);

    registerDLPF = (registerDLPF & 0b11111000) | band; // change only bandwidth

    write8(MPU6050_REGISTER_CONFIG, registerDLPF);
}

void Mpu6050::setSampleRateDivider(byte divider)
{
    write8(MPU6050_REGISTER_SMPRT_DIV, divider);
}

Mpu6050AccelerometerRange Mpu6050::getAccelerometerRange()
{
    return m_accelerometerRange;
}

Mpu6050GyroscopeRange Mpu6050::getGyroscopeRange()
{
    return m_gyroscopeRange;
}

Mpu6050DLPFBandwidth Mpu6050::getDLPFBandwidth()
{
    byte registerDLPF = read8(MPU6050_REGISTER_CONFIG);
    registerDLPF &= 0b00000111; // Keep only the value of DLPF_CFG

    return static_cast<Mpu6050DLPFBandwidth>(registerDLPF);
}

byte Mpu6050::getSampleRateDivider()
{
    return read8(MPU6050_REGISTER_SMPRT_DIV);
}

void Mpu6050::reset()
{
    write8(MPU6050_REGISTER_PWR_MGMT_1, 0b10000000);
}

bool Mpu6050::isConnected()
{
    // The content of WHO_AM_I is always 0x68, so if the wiring is right and
    // the I2C communication works fine this function should return true
    return read8(MPU6050_REGISTER_WHO_AM_I) == MPU6050_DEFAULT_ADDRESS;
}

void Mpu6050::sleepMode()
{
    write8(MPU6050_REGISTER_PWR_MGMT_1, 0b01000000);
}

void Mpu6050::wakeUp()
{
    write8(MPU6050_REGISTER_PWR_MGMT_1, 0b00000000);
}

void Mpu6050::set_ad0(bool ad0)
{
    m_address = MPU6050_DEFAULT_ADDRESS + ad0;
}

bool Mpu6050::get_ad0()
{
    if (m_address == MPU6050_DEFAULT_ADDRESS)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Mpu6050::setBus(TwoWire *bus)
{
    m_wire = bus;
}

TwoWire* Mpu6050::getBus()
{
    return m_wire;
}

float Mpu6050::rawTemperatureToCelsius(int16_t rawTemperature)
{
    return rawTemperature * MPU6050_TEMP_LINEAR_COEF + MPU6050_TEMP_OFFSET;
}

float Mpu6050::rawGyroscopeToDps(int16_t rawGyroscope)
{
    switch (m_gyroscopeRange)
    {
        case Max250Dps:
            return rawGyroscope * MPU6050_GYRO_FACTOR_250;
            break;
        case Max500Dps:
            return rawGyroscope * MPU6050_GYRO_FACTOR_500;
            break;
        case Max1000Dps:
            return rawGyroscope * MPU6050_GYRO_FACTOR_1000;
            break;
        case Max2000Dps:
            return rawGyroscope * MPU6050_GYRO_FACTOR_2000;
            break;
        default:
            break;
    }
}

float Mpu6050::rawAccelerationToMps2(int16_t rawAcceleration)
{
    switch (m_accelerometerRange)
    {
        case Max2g:
            return rawAcceleration * MPU6050_ACCEL_FACTOR_2;
            break;
        case Max4g:
            return rawAcceleration * MPU6050_ACCEL_FACTOR_4;
            break;
        case Max8g:
            return rawAcceleration * MPU6050_ACCEL_FACTOR_8;
            break;
        case Max16g:
            return rawAcceleration * MPU6050_ACCEL_FACTOR_16;
            break;
        default:
            break;
    }
}

// Read one register
byte Mpu6050::read8(byte registerAddr)
{
    m_wire->beginTransmission(m_address);
    m_wire->write(registerAddr);
    m_wire->endTransmission(false);
    m_wire->requestFrom(m_address,1,true);

    return m_wire->read();
}

// read a value contained in two consecutive registers
int16_t Mpu6050::read16(byte registerAddr)
{
    m_wire->beginTransmission(m_address);
    m_wire->write(registerAddr);
    m_wire->endTransmission(false);
    m_wire->requestFrom(m_address,2,true);

    // Concatenate the two bytes
    return m_wire->read()<<8 | m_wire->read();
}

// write one register
byte Mpu6050::write8(byte registerAddr, byte value)
{
    m_wire->beginTransmission(m_address);
    m_wire->write(registerAddr);
    m_wire->write(value);
    m_wire->endTransmission(true);
}
