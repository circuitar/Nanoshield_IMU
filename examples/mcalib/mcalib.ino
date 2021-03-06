/**
 * @file FifoMode.ino
 * Sends magnetometer values in JSON representation to serial. Used with mcalib
 * chorme app to calculate magnetometer calibration parameters.
 * 
 * Copyright (c) 2015 Circuitar
 * This software is released under the MIT license. See the attached LICENSE file for details.
 */
#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU imu;

float x;
float y;
float z;

void setup() {
  Serial.begin(115200);

  imu.setMagnetometerFullScale(LSM303D_MFS_2GAUSS);
  imu.setMagnetometerDataRate(LSM303D_MODR_25);
  imu.setAccelerometerPowerDown();
  imu.begin();

  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
}

void loop() {
  if(imu.magnetHasNewData()) {
    x = imu.readMagnetX();
    y = imu.readMagnetY();
    z = imu.readMagnetZ();
    Serial.print("{\"x\":");
    Serial.print(x, 4);
    Serial.print(",\"y\":");
    Serial.print(y, 4);
    Serial.print(",\"z\":");
    Serial.print(z, 4);
    Serial.print("}\n");
  }
} 