#include <Nanoshield_IMU.h>

#define INT16_T_TOP (32767.0)

Nanoshield_IMU::Nanoshield_IMU(int addr) {
  accelAddress = LSM303D_ACCEL_ADDRESS;
  accelAddress |= addr;

  hasBegun = false;
  actualScale = 2;

  regCtrl0 = 0;
  regCtrl1 = 0 | LSM303D_AODR_1600  // Accelerometer data rate 1600Hz.
               | LSM303D_BDU        // Waits a read operation ends before updates a output register.
               | LSM303D_AZEN       // Accelerometer Z axis enabled.
               | LSM303D_AYEN       // Accelerometer Y axis enabled.
               | LSM303D_AXEN;      // Accelerometer X axis enabled.
  regCtrl2 = 0 | LSM303D_ABW_773    // Accelerometer anti-alias filter bandwidth 773Hz.
               | LSM303D_AFS_2G;    // Accelerometer full-scale +/- 2g.
  regCtrl3 = 0;
  regCtrl4 = 0;
  regCtrl5 = 0;
  regCtrl6 = 0;
  regCtrl7 = 0;
}

void Nanoshield_IMU::begin() {
  Wire.begin();
  writeToAccelerometerRegister(LSM303D_CTRL0, regCtrl0);
  writeToAccelerometerRegister(LSM303D_CTRL1, regCtrl1);
  writeToAccelerometerRegister(LSM303D_CTRL2, regCtrl2);
  writeToAccelerometerRegister(LSM303D_CTRL3, regCtrl3);
  writeToAccelerometerRegister(LSM303D_CTRL4, regCtrl4);
  writeToAccelerometerRegister(LSM303D_CTRL5, regCtrl5);
  writeToAccelerometerRegister(LSM303D_CTRL6, regCtrl6);
  writeToAccelerometerRegister(LSM303D_CTRL7, regCtrl7);
  hasBegun = true;
}

float Nanoshield_IMU::readAccelX() {
  int16_t xAccel = readFromAccelerometerRegister(LSM303D_OUT_X_H_A) << 8;
  xAccel |= readFromAccelerometerRegister(LSM303D_OUT_X_L_A);
  return (float) xAccel * actualScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelY() {
  int16_t yAccel = readFromAccelerometerRegister(LSM303D_OUT_Y_H_A) << 8;
  yAccel |= readFromAccelerometerRegister(LSM303D_OUT_Y_L_A);
  return (float) yAccel * actualScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelZ() {
  int16_t zAccel = readFromAccelerometerRegister(LSM303D_OUT_Z_H_A) << 8;
  zAccel |= readFromAccelerometerRegister(LSM303D_OUT_Z_L_A);
  return (float) zAccel * actualScale / INT16_T_TOP;
}

void Nanoshield_IMU::writeToAccelerometerRegister(int8_t reg, int8_t value) {
  Wire.beginTransmission(accelAddress);
  Wire.write(reg);
  Wire.write(value);
  i2cError = Wire.endTransmission();
}

inline void Nanoshield_IMU::writeIfHasBegun(int8_t reg, int8_t value) {
  if(hasBegun) {
    writeToAccelerometerRegister(reg, value);
  }
}

int Nanoshield_IMU::readFromAccelerometerRegister(int8_t reg) {
  Wire.beginTransmission(accelAddress);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(accelAddress, 1);
  return Wire.read();
}