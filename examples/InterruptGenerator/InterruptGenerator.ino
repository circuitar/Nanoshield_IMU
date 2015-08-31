#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <PinChangeInterrupt.h>
#include <util/atomic.h>

#define INTERRUPT_PIN 6

void movement();

Nanoshield_IMU imu;
bool movementDetected = false;

void setup() {
  Serial.begin(9600);

  // Set the interrupt generator to detect movement:
  imu.setAccelIntGenerator1Mode(LSM303D_INTMODE_MOVEMENT);

  // Add desired zones to interrupt generator:
  imu.addToAccelIntGenerator1Zone(LSM303D_ZONE_ZH);
  imu.addToAccelIntGenerator1Zone(LSM303D_ZONE_YH);
  imu.addToAccelIntGenerator1Zone(LSM303D_ZONE_YL);
  imu.addToAccelIntGenerator1Zone(LSM303D_ZONE_XH);
  imu.addToAccelIntGenerator1Zone(LSM303D_ZONE_XL);

  // Define a threshold to all specified zones:
  imu.setAccelIntGenerator1Threshold(0.6);
  
  imu.begin();

  // Bind interrupt generator 1 to interruption pin 1 (D6)
  imu.setInterrupt1Source(LSM303D_INT1_IG1);

  pinMode(INTERRUPT_PIN, INPUT);
  attachPCINT(digitalPinToPCINT(INTERRUPT_PIN), movement, RISING);
}

void loop() {
  bool movDetected = false;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    movDetected = movementDetected;
  }

  if(movDetected) {
    int intStatus = imu.getAccelIntGenerator1Status();

    if((intStatus & LSM303D_ZONE_ZH) != 0) {
      Serial.print("Movement on Z: ");
      Serial.print(imu.readAccelZ());
      Serial.println("g");
    }

    if((intStatus & (LSM303D_ZONE_YH | LSM303D_ZONE_YL)) != 0) {
      Serial.print("Movement on Y: ");
      Serial.print(imu.readAccelY());
      Serial.println("g");
    }

    if((intStatus & (LSM303D_ZONE_XH | LSM303D_ZONE_XL)) != 0) {
      Serial.print("Movement on X: ");
      Serial.print(imu.readAccelX());
      Serial.println("g");
    }

    Serial.println();

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      movementDetected = false;
    }
  }
}

void movement() {
  movementDetected = true;
}