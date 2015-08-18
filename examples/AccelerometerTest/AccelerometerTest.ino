#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU imu;
bool pwrdown = false;
bool pwrup = false;
float selftest[3];

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.print("Accelerometer test.\n\n");
  imu.setAccelerometerDataRate(LSM303D_AODR_50);
  imu.begin();

  if(imu.selfTest(selftest)) {
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
  Serial.println("g");

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

  delay(500);
}
