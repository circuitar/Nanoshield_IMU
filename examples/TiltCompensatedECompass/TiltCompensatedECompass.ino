#include <Wire.h>
#include <Nanoshield_IMU.h>
#include <math.h>

// http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf

Nanoshield_IMU imu;
int16_t centerOffset[3];
int i = 0;

void setup() {
  imu.setAccelerometerDataRate(LSM303D_AODR_50);
  imu.setMagnetometerDataRate(LSM303D_MODR_50);
  imu.begin();

  centerOffset[0] = 0;
  centerOffset[1] = 0;
  centerOffset[2] = 0;

  Serial.begin(9600);
}

void loop() {
  if(i < 1000 && imu.magnetHasNewData()) {
    centerOffset[0] += imu.magnetRawX();
    centerOffset[1] += imu.magnetRawY();
    centerOffset[2] += imu.magnetRawZ();
    i++;
  }

  if(i == 1000) {
    centerOffset[0] /= i;
    centerOffset[1] /= i;
    centerOffset[2] /= i;
  }
  
}