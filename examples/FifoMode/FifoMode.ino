#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>

#define FTH_INT_PIN 5
#define EMPTY_INT_PIN 6

void bufferFull();
void bufferEmpty();

Nanoshield_IMU imu;
long timesec = 0, tsec= 0;
bool bufferStatus = false;
bool resetBuffer = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Data Ready Interruption\n");

  pinMode(FTH_INT_PIN, INPUT);
  pinMode(EMPTY_INT_PIN, INPUT);
  
  pinMode(13, OUTPUT);

  imu.setAccelerometerDataRate(LSM303D_AODR_100);
  imu.enableAccelBuffer(LSM303D_FIFO, 19);
  imu.begin();

  imu.setInterrupt1Source(LSM303D_INT1_EMPTY);
  imu.setInterrupt2Source(LSM303D_INT2_FTH);

  attachPCINT(digitalPinToPCINT(FTH_INT_PIN), bufferFull, RISING);
  attachPCINT(digitalPinToPCINT(EMPTY_INT_PIN), bufferEmpty, RISING);
}

void loop() {
  digitalWrite(13, digitalRead(FTH_INT_PIN));
  bool bStatus = false;
  bool rstBuffer = false;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    bStatus = bufferStatus;
    rstBuffer = resetBuffer;
  }

  if(bStatus) {
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
    Serial.println();
    Serial.println("-----------------------");
    Serial.println();

    imu.resetAccelBuffer();

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      resetBuffer = false;
    }
  }
}

void bufferFull() {
  bufferStatus = true;
}

void bufferEmpty() {
  bufferStatus = false;
  resetBuffer = true;
}