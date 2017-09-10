/* *****************************************************************************
 * Filename: readMeasurements.ino
 * 
 * Description: This is a simple example for the Simple-MPU6050-Arduino library.
 * The acceleration is read over and over, and the built-in LED is turned on 
 * whenever the acceleration is "strong".
 * 
 * Author: Thomas Havy
 * 
 * License: MIT License - Copyright (c) 2017 Thomas Havy
 * A copy of the license can be found at:
 * https://github.com/Th-Havy/Simple-MPU6050-Arduino/blob/master/LICENSE
 * 
 * Changes:
 * - Created: 10-Sept-2017
 * 
 * ************************************************************************* */

// Include the library
#include <Mpu6050.h>

// Global variables
Mpu6050 sensor;
Vector3f accel;

void setup() 
{
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  // Add sensor and then initialize it
  sensor = Mpu6050();

  if (sensor.init())
  {
    Serial.println("Mpu6050 Connected!");
  }
  else
  {
    Serial.println("Failed to connect to Mpu6050.");
  }
}

void loop()
{
  // Collect the current sensor measurement
  accel = sensor.readAcceleration();

  // Turn on LED if the acceleration is intense
  if (accel.magnitude() > 15.0)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

