/**
 * @file FifoMode.ino
 * Lets arduino sleeping while waits for Nanoshield IMU data.
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
#include <avr/sleep.h>

#define FTH_INT_PIN 5
#define EMPTY_INT_PIN 6

void bufferFull();
void bufferEmpty();
void sleep();

Nanoshield_IMU imu;
long timesec = 0, tsec= 0;
bool bufferReady = false;
bool resetBuffer = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Data Ready Interruption\n");

  
  pinMode(13, OUTPUT);

  imu.setAccelerometerDataRate(LSM303D_AODR_12_5); // Work at 12.5Hz data rate.
  imu.setMagnetometerPowerDown(); // Turn magnetometer off.
  // Enable accelerometer buffer in FIFO mode with a threshold of 20 positions.
  imu.enableAccelBuffer(LSM303D_FIFO, 19);
  imu.begin();

  imu.setInterrupt1Source(LSM303D_INT1_EMPTY);
  imu.setInterrupt2Source(LSM303D_INT2_FTH);

  pinMode(FTH_INT_PIN, INPUT);
  pinMode(EMPTY_INT_PIN, INPUT);

  attachPCINT(digitalPinToPCINT(FTH_INT_PIN), bufferFull, RISING);
  attachPCINT(digitalPinToPCINT(EMPTY_INT_PIN), bufferEmpty, RISING);

  sleep();
}

void loop() {
  bool bReady = false;
  bool rstBuffer = false;

  // Read interruption flags atomically to avoid concurrency.
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    bReady = bufferReady;
    rstBuffer = resetBuffer;
  }

  if(bReady) {
    Serial.print("AccelX: ");
    Serial.print(imu.readAccelX());
    Serial.println("g");

    Serial.print("AccelY: ");
    Serial.print(imu.readAccelY());
    Serial.println("g");

    Serial.print("AccelZ: ");
    Serial.print(imu.readAccelZ());
    Serial.println("g\n");
  }

  if(rstBuffer) {
    Serial.println("\n-----------------------\n");

    imu.resetAccelBuffer();

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      resetBuffer = false;
    }

    sleep();
  }
}

void bufferFull() {
  bufferReady = true;
}

void bufferEmpty() {
  bufferReady = false;
  resetBuffer = true;
}

void sleep() {
  Serial.flush(); // Waits Serial to send any data before sleep.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable(); // Enables atmega sleep feature
  sleep_mode();   // Puts atmega to sleep.

  // Zzzzz

  sleep_disable();
}