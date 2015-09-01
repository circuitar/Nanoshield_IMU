/**
 * @file DataReadyInterruption.ino
 * Reads the accelerometer and magnetometer asynchronously through the data
 * ready interruption. Demonstrates a way to attach interruption to any arduino
 * pin using the PinChangeInterrupt library.
 * 
 * Copyright (c) 2015 Circuitar
 * This software is released under the MIT license. See the attached LICENSE file for details.
 */
#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <util/atomic.h>
/*
 * To install the PinChangeInterrupt library download the library as zip from
 * this link <https://github.com/NicoHood/PinChangeInterrupt>. After it, in
 * Arduino IDE, select Sketch > Include library > Add .ZIP library.
 */
#include <PinChangeInterrupt.h>

#define ACCEL_DRDY_PIN 6  // Pin used to accelerometer data ready interruption.
#define MAGNET_DRDY_PIN 5 // Pin used to magnetometer data ready interruption.

void accelDataReady();
void magnetDataReady();
void gyroDataReady();

Nanoshield_IMU imu;
bool accelerometerReady = false;
bool magnetometerReady = false;
bool gyroscopeReady = false;
long lastAccelData = 0;
long lastMagnetData = 0;
long lastGyroData = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Data Ready Interruption\n");

  // Setup both accel and magnetometer drdy (data ready) pins as input.
  pinMode(ACCEL_DRDY_PIN, INPUT);
  pinMode(MAGNET_DRDY_PIN, INPUT);

  // The data ready interruption just works with 3.125Hz and 6.25Hz.
  imu.setAccelerometerDataRate(LSM303D_AODR_3_125);
  imu.setMagnetometerDataRate(LSM303D_MODR_3_125);
  imu.setGyroscopeDataRate(L3GD20H_DRBW_12_5);
  imu.begin(); // Post configuration to Nanoshield.
  imu.setInterrupt1Source(LSM303D_INT1_DRDY_A); // Set INT1 as accel drdy.
  imu.setInterrupt2Source(LSM303D_INT2_DRDY_M); // Set INT2 as magnet drdy.
  imu.setGyroInterruptSource(L3GD20H_INT2_DRDY);

  // Attach the interruption handlers to its interruptions pin.
  attachPCINT(digitalPinToPCINT(ACCEL_DRDY_PIN), accelDataReady, RISING);
  attachPCINT(digitalPinToPCINT(MAGNET_DRDY_PIN), magnetDataReady, RISING);

  // As D2 is the gyroscope interrupt pin, it can be attached to a normal
  // interrupt. Not need to use a pcint.
  attachInterrupt(0, gyroDataReady, RISING);
}

void loop() {
  // Local flags to avoid concurrency with interruptions.
  bool accelReady = false;
  bool magnetReady = false;
  bool gyroReady = false;

  // Atomic reading to avoid concurrency with the interruptions.
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    accelReady = accelerometerReady;
    magnetReady = magnetometerReady;
    gyroReady = gyroscopeReady;
  }

  // Handles accel drdy interruption.
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

  // Handles magnet drdy interruption.
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

  if(gyroReady) {
    Serial.print("Period: ");
    Serial.print(millis() - lastMagnetData);
    lastMagnetData = millis();
    Serial.println("ms");

    Serial.print("GyroX: ");
    Serial.print(imu.readGyroX());
    Serial.println("dps");

    Serial.print("GyroY: ");
    Serial.print(imu.readGyroY());
    Serial.println("dps");

    Serial.print("GyroZ: ");
    Serial.print(imu.readGyroZ());
    Serial.println("dps\n");

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      gyroscopeReady = false;
    }
  }
}

// Sets a flag to handle an accel drdy interruption.
void accelDataReady() {
  accelerometerReady = true;
}

// Sets a flag to handle a magnet drdy interruption.
void magnetDataReady() {
  magnetometerReady = true;
}

// Sets a flag to handle a gyro drdy interruption.
void gyroDataReady() {
  gyroscopeReady = true;
}