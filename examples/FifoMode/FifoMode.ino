#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <PinChangeInterrupt.h>
#include <util/atomic.h>
//#include <avr/sleep.h>

#define FIFO_INT_PIN 5
#define ACCEL_INT_PIN 6

void sleep();
void wakeup();
void accelInt();

Nanoshield_IMU imu;
bool bufferFull = false;
long lastBlink = 0;
int accelCount = 0;

void setup() {
  pinMode(FIFO_INT_PIN, INPUT);
  pinMode(ACCEL_INT_PIN, INPUT);
  pinMode(13, OUTPUT);

  imu.setAccelerometerDataRate(LSM303D_AODR_3_125);
  imu.setMagnetometerDataRate(LSM303D_MODR_3_125);
  // To save energy, turn the magnetometer off.
  // imu.setMagnetometerPowerDown();
  // Post the configuration to Nanoshield.
  imu.begin();

  // Set buffer to operate in FIFO mode.
  // As buffer threshold not specified, using default value: 31.
  imu.setAccelBufferMode(LSM303D_FIFO);
  imu.setInterrupt1Source(LSM303D_INT1_DRDY_A);
  imu.setInterrupt2Source(LSM303D_INT2_FTH);

  Serial.begin(9600);
  Serial.println("Waiting buffer to fill...");
  Serial.println();
  Serial.println(imu.readFromLSM303DRegister(LSM303D_CTRL3));
  Serial.println(imu.readFromLSM303DRegister(LSM303D_CTRL4));
  Serial.println(imu.readFromLSM303DRegister(LSM303D_FIFO_CTRL));
  Serial.println();
  Serial.flush();
  Serial.end();

  attachPCINT(digitalPinToPCINT(FIFO_INT_PIN), wakeup, RISING);
  attachPCINT(digitalPinToPCINT(ACCEL_INT_PIN), accelInt, RISING);

  //attachInterrupt(0, wakeup, FALLING);

  //sleep();
}

void loop() {
  bool bFull = false;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    bFull = bufferFull;
  }

  if(bFull) {
    Serial.println("Buffer full!");
    Serial.println();
    Serial.flush();
    Serial.end();

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      bufferFull = false;
    }
  }

  if(accelCount > 30) {
    Serial.println("accelCount: ");
    Serial.println(accelCount);
    accelCount = 0;
  }

  digitalWrite(13, digitalRead(ACCEL_INT_PIN));

  //sleep();
}

/*
void sleep() {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  attachPCINT(digitalPinToPCINT(FIFO_INT_PIN), wakeup, FALLING);
  attachInterrupt(0, wakeup, LOW);
  sleep_mode();

  // Zzzz

  sleep_disable();
  detachPCINT(digitalPinToPCINT(FIFO_INT_PIN));
  detachInterrupt(0);
}
*/

void wakeup() {
  bufferFull = true;
}

void accelInt() {
  accelCount++;
}