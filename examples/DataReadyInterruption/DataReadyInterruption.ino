#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#define INTERRUPT_PIN 10 // D6 pin

void dataReady();

Nanoshield_IMU imu;
bool dataReadyInterruption = false;
bool accelerometerReady = false;
bool magnetometerReady = false;
long lastAccelData = 0;
long lastMagnetData = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Data Ready Interruption\n");

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(11, INPUT);
  digitalWrite(11, HIGH);
  pinMode(13, OUTPUT);

  attachPCINT(digitalPinToPCINT(11), dataReady, FALLING);
  attachPCINT(digitalPinToPCINT(INTERRUPT_PIN), dataReady, RISING);

  imu.setAccelerometerDataRate(LSM303D_AODR_3_125);
  imu.setInterrupt1Source(LSM303D_INT1_DRDY_A);
  imu.begin();
}

void loop() {
  digitalWrite(13, digitalRead(INTERRUPT_PIN));
  bool accelReady = false;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    accelReady = accelerometerReady;
  }


  if(accelReady) {
    //Serial.println(imu.accelHasNewData());
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
}

void dataReady() {
  accelerometerReady = true;
}