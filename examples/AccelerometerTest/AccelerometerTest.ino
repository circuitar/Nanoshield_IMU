#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU accel;

void setup() {
  Serial.begin(9600);
  Serial.print("Accelerometer test.\n\n");
  accel.begin();
  Serial.println(accel.accelAddress);
}

void loop() {
  float a = accel.readAccelX();
  Serial.print("AccelX: ");
  Serial.print(a);
  Serial.println("g");

  a = accel.readAccelY();
  Serial.print("AccelY: ");
  Serial.print(a);
  Serial.println("g");

  a = accel.readAccelZ();
  Serial.print("AccelZ: ");
  Serial.print(a);
  Serial.println("g\n");

  delay(1000);
}
