#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>

#define ACCEL_DRDY_PIN 6
#define MAGNET_DRDY_PIN 5

void accelDataReady();
void magnetDataReady();

Nanoshield_IMU imu;
bool accelerometerReady = false;
bool magnetometerReady = false;
long lastAccelData = 0;
long lastMagnetData = 0;
long lastBlink = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Data Ready Interruption\n");

  pinMode(ACCEL_DRDY_PIN, INPUT);
  pinMode(MAGNET_DRDY_PIN, INPUT);
  
  pinMode(13, OUTPUT);


  imu.setAccelerometerDataRate(LSM303D_AODR_3_125);
  imu.setMagnetometerDataRate(LSM303D_MODR_3_125);
  imu.begin();
  imu.setInterrupt1Source(LSM303D_INT1_DRDY_A);
  imu.setInterrupt2Source(LSM303D_INT2_DRDY_M);

  attachPCINT(digitalPinToPCINT(ACCEL_DRDY_PIN), accelDataReady, RISING);
  attachPCINT(digitalPinToPCINT(MAGNET_DRDY_PIN), magnetDataReady, RISING);
}

void loop() {
  bool accelReady = false;
  bool magnetReady = false;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    accelReady = accelerometerReady;
    magnetReady = magnetometerReady;
  }


  if(accelReady) {
    
    Serial.print("Period: ");
    Serial.print(millis() - lastAccelData);
    lastAccelData = millis();
    Serial.println("ms");

    Serial.print("AccelX: ");
    Serial.print(imu.readAccelX());
    Serial.println("g");

    Serial.print("AccelY: ");
    Serial.print(imu.readAccelY());
    Serial.println("g");

    Serial.print("AccelZ: ");
    Serial.print(imu.readAccelZ());
    Serial.println("g\n");

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      accelerometerReady = false;
    }
  }

  if(magnetReady) {
    Serial.print("Period: ");
    Serial.print(millis() - lastMagnetData);
    lastMagnetData = millis();
    Serial.println("ms");

    Serial.print("MagnetX: ");
    Serial.print(imu.readMagnetX());
    Serial.println("gauss");

    Serial.print("MagnetY: ");
    Serial.print(imu.readMagnetY());
    Serial.println("gauss");

    Serial.print("MagnetZ: ");
    Serial.print(imu.readMagnetZ());
    Serial.println("gauss\n");

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      magnetometerReady = false;
    }
  }

  if(millis() - lastBlink > 500) {
    digitalWrite(13, !digitalRead(13));
    lastBlink = millis();
  }
}

void accelDataReady() {
  accelerometerReady = true;
}

void magnetDataReady() {
  magnetometerReady = true;
}