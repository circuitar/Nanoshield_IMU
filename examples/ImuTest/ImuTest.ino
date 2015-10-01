/**
 * @brief ImuTest.ino
 * A simple test to print on serial the measures from accelerometer and
 * magnetometer.
 * 
 * Copyright (c) 2015 Circuitar
 * This software is released under the MIT license. See the attached LICENSE file for details.
 */
#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU imu;

void setup() {
  Serial.begin(9600);
  Serial.print("Accelerometer test.\n\n");
  imu.begin();
}

void loop() {
  Serial.println("----------------\n");

  if(imu.accelHasNewData()) {
    Serial.print("AccelX: ");
    Serial.print(imu.readAccelX());
    Serial.println("g");

    Serial.print("AccelY: ");
    Serial.print(imu.readAccelY());
    Serial.println("g");

    Serial.print("AccelZ: ");
    Serial.print(imu.readAccelZ());
    Serial.println("g\n");
  }

  if(imu.magnetHasNewData()){
      Serial.print("MagnetX: ");
      Serial.print(imu.readMagnetX());
      Serial.println("gauss");
  
      Serial.print("MagnetX: ");
      Serial.print(imu.readMagnetY());
      Serial.println("gauss");
  
      Serial.print("MagnetX: ");
      Serial.print(imu.readMagnetZ());
      Serial.println("gauss");

      Serial.println("Heading: ");
      Serial.print(imu.heading());
      Serial.println("rad\n");
  }

  if(imu.gyroHasNewData()) {
      Serial.print("GyroX: ");
      Serial.print(imu.readGyroX());
      Serial.println("dps");
  
      Serial.print("GyroY: ");
      Serial.print(imu.readGyroY());
      Serial.println("dps");
  
      Serial.print("GyroZ: ");
      Serial.print(imu.readGyroZ());
      Serial.println("dps\n");
  }
}
