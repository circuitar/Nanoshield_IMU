#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>

#define FIFO_DRDY_PIN 5

void bufferFull();

Nanoshield_IMU imu;
bool accelerometerReady = false;
bool bufferReady = false;
long lastAccelData = 0;
long lastMagnetData = 0;
long lastBlink = 0;
int accelCount = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Data Ready Interruption\n");
  
  pinMode(FIFO_DRDY_PIN, INPUT);
  
  pinMode(13, OUTPUT);


  imu.setAccelerometerDataRate(LSM303D_AODR_3_125);
  imu.enableAccelBuffer();
  imu.begin();
  imu.setInterrupt2Source(LSM303D_INT2_FTH);

  attachPCINT(digitalPinToPCINT(FIFO_DRDY_PIN), bufferFull, RISING);
}

void loop() {
  digitalWrite(13, digitalRead(FIFO_DRDY_PIN));
  bool accelReady = false;
  bool bufReady = false;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    bufReady = bufferReady;
  }

  if(bufReady) {
    Serial.println("bufferFull!");
    Serial.println(imu.getBufferCount());

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      bufferReady = false;
    }
  }
}

void bufferFull() {
  bufferReady = true;
}