/**
 * @file InterruptGenerator.ino
 * Example of accelerometer interruptio generator usage.
 * 
 * Copyright (c) 2015 Circuitar
 * This software is released under the MIT license. See the attached LICENSE file for details.
 */
#include <Wire.h>
#include <Nanoshield_IMU.h>
/*
 * To install the PinChangeInterrupt library download the library as zip from
 * this link <https://github.com/NicoHood/PinChangeInterrupt>. After it, in
 * Arduino IDE, select Sketch > Include library > Add .ZIP library.
 */
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

  // Turn magnetometer off to save energy.
  imu.setMagnetometerPowerDown();
  
  imu.begin();

  // Bind interrupt generator 1 to interruption pin 1 (D6)
  imu.setInterrupt1Source(LSM303D_INT1_IG1);

  pinMode(INTERRUPT_PIN, INPUT);
  attachPCINT(digitalPinToPCINT(INTERRUPT_PIN), movement, RISING);
}

void loop() {
  bool movDetected = false;

  // Read interruption flag atomically to avoid concurrency.
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    movDetected = movementDetected;
  }

  if(movDetected) {
    // Check the interrupt generator status to find what happened.
    int intStatus = imu.getAccelIntGenerator1Source();

    // Check if movement in Z positive axis.
    if((intStatus & LSM303D_ZONE_ZH) != 0) {
      Serial.print("Movement on Z: ");
      Serial.print(imu.readAccelZ());
      Serial.println("g");
    }

    // Check if movement in Y positive or negative axes.
    if((intStatus & (LSM303D_ZONE_YH | LSM303D_ZONE_YL)) != 0) {
      Serial.print("Movement on Y: ");
      Serial.print(imu.readAccelY());
      Serial.println("g");
    }

    // Check if movement in X positive or negative axes.
    if((intStatus & (LSM303D_ZONE_XH | LSM303D_ZONE_XL)) != 0) {
      Serial.print("Movement on X: ");
      Serial.print(imu.readAccelX());
      Serial.println("g");
    }

    Serial.println();

    // Atomically reset interruption flag.
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      movementDetected = false;
    }
  }
}

void movement() {
  movementDetected = true;
}