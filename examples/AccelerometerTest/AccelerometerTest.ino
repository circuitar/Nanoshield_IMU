#include <Wire.h>
#include <Nanoshield_IMU.h>

Nanoshield_IMU accel;
bool pwrdown = false;
bool pwrup = false;

void setup() {
  Serial.begin(9600);
  Serial.print("Accelerometer test.\n\n");
  accel.begin();
  pinMode(13, OUTPUT);
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

  if(millis() > 10000 and !pwrdown) {
    digitalWrite(13, HIGH);
    accel.disableAccelXAxis();
    pwrdown = true;
  }

  if(millis() > 20000 and !pwrup) {
    digitalWrite(13, LOW);
    accel.enableAccelXAxis();
    pwrup = true;
  }
}
