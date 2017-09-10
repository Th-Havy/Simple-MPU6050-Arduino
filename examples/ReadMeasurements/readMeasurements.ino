/* *****************************************************************************
 * Filename: readMeasurements.ino
 * 
 * Description: This is a simple example for the Simple-MPU6050-Arduino library.
 * It shows how to setup the sensor and how to read the acceleration, gyroscope
 * and temperature measurements. Each second, all these values are sent to the
 * Serial port, and can be displyed in the Serial monitor: in the Arduino IDE,
 * go to Tools > Serial Monitor (Ctrl+Maj+m).
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
Mpu6050Data data;

void setup() 
{
  Serial.begin(9600);

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
  // Collect the current sensor measurements
  data = sensor.readData();

  // 
  Serial.println("---------------------------------------");
  Serial.print("Acceleration (m/s^2): X="); Serial.print(data.acceleration.x); Serial.print(" Y="); Serial.print(data.acceleration.y); Serial.print(" Z="); Serial.println(data.acceleration.z);
  Serial.print("Gyroscope (°/s): X="); Serial.print(data.gyroscope.x); Serial.print(" Y="); Serial.print(data.gyroscope.y); Serial.print(" Z="); Serial.println(data.gyroscope.z);
  Serial.print("Temperature (°C): "); Serial.println(data.temperature);

  delay(1000);
}

