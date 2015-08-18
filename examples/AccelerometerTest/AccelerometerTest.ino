#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU accel;
bool pwrdown = false;
bool pwrup = false;
float selftest[3];

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.print("Accelerometer test.\n\n");
  accel.setAccelerometerDataRate(LSM303D_AODR_50);
  accel.begin();

  if(accel.selfTest(selftest)) {
    digitalWrite(13, HIGH);
  }
  
  Serial.print("selftest x: ");
  Serial.println(selftest[0]);
  Serial.print("selftest y: ");
  Serial.println(selftest[1]);
  Serial.print("selftest z: ");
  Serial.println(selftest[2]);
  Serial.println();
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

  delay(500);
}
