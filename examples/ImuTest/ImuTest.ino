/**
 * @brief ImuTest.ino
 * A simple test to print on serial the measures from accelerometer and
 * magnetometer.
 */
#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU imu;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.print("Accelerometer test.\n\n");
  imu.begin();
}

void loop() {
  Serial.println("----------------\n");

  float a = imu.readAccelX();
  Serial.print("AccelX: ");
  Serial.print(a);
  Serial.println("g");

  a = imu.readAccelY();
  Serial.print("AccelY: ");
  Serial.print(a);
  Serial.println("g");

  a = imu.readAccelZ();
  Serial.print("AccelZ: ");
  Serial.print(a);
  Serial.println("g\n");

  a = imu.readMagnetX();
  Serial.print("MagnetX: ");
  Serial.print(a);
  Serial.println("gauss");

  a = imu.readMagnetY();
  Serial.print("MagnetX: ");
  Serial.print(a);
  Serial.println("gauss");

  a = imu.readMagnetZ();
  Serial.print("MagnetX: ");
  Serial.print(a);
  Serial.println("gauss\n");

  a = imu.readGyroX();
  Serial.print("GyroX: ");
  Serial.print(a);
  Serial.println("dps");

  a = imu.readGyroY();
  Serial.print("GyroY: ");
  Serial.print(a);
  Serial.println("dps");

  a = imu.readGyroZ();
  Serial.print("GyroZ: ");
  Serial.print(a);
  Serial.println("dps\n");
  delay(500);
}
