/* *****************************************************************************
 * Filename: fullConfiguration.ino
 * 
 * Description: This is a simple example for the Simple-MPU6050-Arduino library.
 * It shows how to fully configure the sensors.
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

// Constants
const bool ad0 = LOW; // AD0 pin

// Global variables
Mpu6050 sensor;
Mpu6050Data data;

void setup() 
{
  Serial.begin(9600);

  // Add sensor, specifying ad0 and the I2C bus used
  sensor = Mpu6050(ad0, &Wire);

  /* Initialise the sensor specifying all options:
   *  
   * Acceleration range: how much acceleration can be measured,
   * can be +/-2g,4g,8g,16g (use the constans Max2g,Max4g,etc...)
   * You should use the lowest possible range for your need, because
   * you then get more precision on the measurement, but any 
   * acceleration greater than the range will be incorrectly 
   * measured. 
   * 
   * Gyroscope range: how much rotationnal speed can be measured,
   * can be +/-250 °/s,500,1000,2000 (use the constans Max250Dps,
   * Max500Dps,etc...).
   * You should use the lowest possible range for your need,for
   * the same reasons as the acceleration range.
   * 
   * Digital Low Pass Filter (DLPF): this parameter can be use to
   * enable the DLPF, which will (I'm simplifying) average the 
   * measurements on a short time (microseconds to milliseconds)
   * leading to less "jittery" measurements. However, this limits
   * the number of sample readable per second. Use the constants
   * Max260Hz, Max184Hz, Max94Hz, Max44Hz, Max21Hz, Max10Hz, 
   * Max5Hz (number of different samples per second).
   * 
   * Sample Rate Divider: Change the number of sample per second
   * sample rate = 8kHz / (1 + divider), I recommand leaving it
   * with a value of 7 unless you need to synchronize the output
   * rate of the sensor for some reason.
   
   */
  
  if (sensor.init(Max4g, Max500Dps, Max184Hz, 7))
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

  // Print data
  Serial.println("---------------------------------------");
  Serial.print("Acceleration (m/s^2): X="); Serial.print(data.acceleration.x); Serial.print(" Y="); Serial.print(data.acceleration.y); Serial.print(" Z="); Serial.println(data.acceleration.z);
  Serial.print("Gyroscope (°/s): X="); Serial.print(data.gyroscope.x); Serial.print(" Y="); Serial.print(data.gyroscope.y); Serial.print(" Z="); Serial.println(data.gyroscope.z);
  Serial.print("Temperature (°C): "); Serial.println(data.temperature);

  delay(1000);
}

