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

  if(millis() > 10000 && !pwrdown) {
    imu.setMagnetometerPowerDown();
    digitalWrite(13, HIGH);
    pwrdown = true;
  }

  if(millis() > 20000 && !pwrup) {
    imu.setMagnetometerDataRate(LSM303D_M_ODR_100);
    digitalWrite(13, LOW);
    pwrup = true;
  }

  delay(500);
}
