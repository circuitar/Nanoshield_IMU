/**
 * @file Nanoshield_IMU.h
 * 
 * A library to access the Nanoshield IMU v2.0.
 * It uses the LSM303D for accelerometer, magnetometer and temperature.
 * 
 * Copyright (c) 2015 Circuitar
 * This software is released under the MIT license. See the attached LICENSE file for details.
 */
#include <Nanoshield_IMU.h>

#define INT16_T_TOP (32767.0)

Nanoshield_IMU::Nanoshield_IMU(int addr) {
  lsm303dAddress = LSM303D_ADDRESS | addr;

  hasBegun = false;
  accelScale = 2;
  magnetScale = 2;

  regCtrl0 = 0;
  regCtrl1 = 0 | LSM303D_AODR_1600  // Accelerometer data rate 1600Hz.
               | LSM303D_BDU        // Waits a read operation ends before updates a output register.
               | LSM303D_AZEN       // Accelerometer Z axis enabled.
               | LSM303D_AYEN       // Accelerometer Y axis enabled.
               | LSM303D_AXEN;      // Accelerometer X axis enabled.
  regCtrl2 = 0 | LSM303D_ABW_773    // Accelerometer anti-alias filter bandwidth 773Hz.
               | LSM303D_AFS_2G;    // Accelerometer full-scale +/- 2g.
  regCtrl5 = 0 | LSM303D_M_ODR_100;   // Magnetic data rate 100Hz.
  regCtrl6 = 0 | LSM303D_MFS_2GAUSS;  // Magnetic full-scale +/- 2gauss.
  regCtrl7 = 0 | LSM303D_MD_CONTINUOUS; // Magnetometer in continuous mode.
}

void Nanoshield_IMU::begin() {
  Wire.begin();
  writeToLSM303DRegister(LSM303D_CTRL0, regCtrl0);
  writeToLSM303DRegister(LSM303D_CTRL1, regCtrl1);
  writeToLSM303DRegister(LSM303D_CTRL2, regCtrl2);
  writeToLSM303DRegister(LSM303D_CTRL5, regCtrl5);
  writeToLSM303DRegister(LSM303D_CTRL6, regCtrl6);
  writeToLSM303DRegister(LSM303D_CTRL7, regCtrl7);
  writeToLSM303DRegister(LSM303D_INT_CTRL_M, 0 | LSM303D_MIEN);
  readAccelX();
  hasBegun = true;
}

void Nanoshield_IMU::setAccelerometerPowerDown() {
  regCtrl1 &= ~LSM303D_AODR_MASK;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::setAccelerometerDataRate(int8_t drate) {
  regCtrl1 &= ~LSM303D_AODR_MASK;
  regCtrl1 |= drate;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::enableAccelXAxis() {
  regCtrl1 |= LSM303D_AXEN;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::disableAccelXAxis() {
  regCtrl1 &= ~LSM303D_AXEN;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::enableAccelYAxis() {
  regCtrl1 |= LSM303D_AYEN;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::disableAccelYAxis() {
  regCtrl1 &= ~LSM303D_AYEN;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::enableAccelZAxis() {
  regCtrl1 |= LSM303D_AZEN;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::disableAccelZAxis() {
  regCtrl1 &= ~LSM303D_AZEN;

  writeIfHasBegun(LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::setAccelerometerFullScale(int8_t scale) {
  switch(scale) {
    case LSM303D_AFS_2G:
      accelScale = 2;
      break;
    case LSM303D_AFS_4G:
      accelScale = 4;
      break;
    case LSM303D_AFS_6G:
      accelScale = 6;
      break;
    case LSM303D_AFS_8G:
      accelScale = 8;
      break;
    case LSM303D_AFS_16G:
      accelScale = 16;
      break;
    default:
      return;
  }

  regCtrl2 &= ~LSM303D_AFS_MASK;
  regCtrl2 |= scale;

  writeIfHasBegun(LSM303D_CTRL2, regCtrl2);
}

bool Nanoshield_IMU::selfTest(float diff[]) {
  register int8_t localRegCtrl2 = regCtrl2;
  register float measurePeriod;
  float measures[6];

  // Calculate the time between two measures.
  switch(regCtrl1 & LSM303D_AODR_MASK) {
    case LSM303D_AODR_3_125:
      measurePeriod = 1000.0 / 3.125;
      break;
    case LSM303D_AODR_6_25:
      measurePeriod = 1000.0 / 6.25;
      break;
    case LSM303D_AODR_12_5:
      measurePeriod = 1000.0 / 12.5;
      break;
    case LSM303D_AODR_25:
      measurePeriod = 1000.0 / 25;
      break;
    case LSM303D_AODR_50:
      measurePeriod = 1000.0 / 50;
      break;
    case LSM303D_AODR_100:
      measurePeriod = 1000.0 / 100;
      break;
    case LSM303D_AODR_200:
      measurePeriod = 1000.0 / 200;
      break;
    case LSM303D_AODR_400:
      measurePeriod = 1000.0 / 400;
      break;
    case LSM303D_AODR_800:
      measurePeriod = 1000.0 / 800;
      break;
    case LSM303D_AODR_1600:
      measurePeriod = 1000.0 / 1600;
      break;
    default:
      return false;
  }

  // Sets full scale to +/- 2g,
  // sets the self test flag
  // and wait next measure to be ready.
  localRegCtrl2 &= ~LSM303D_AFS_MASK;
  localRegCtrl2 |= LSM303D_AFS_2G; 
  localRegCtrl2 |= LSM303D_AST;
  writeToLSM303DRegister(LSM303D_CTRL2, localRegCtrl2);
  delay(measurePeriod);

  // Read the simulated acceleration.
  measures[0] = readAccelX();
  measures[1] = readAccelY();
  measures[2] = readAccelZ();

  // Unset the self test flag
  // and wait next measure to be ready.
  localRegCtrl2 &= ~LSM303D_AST;
  writeToLSM303DRegister(LSM303D_CTRL2, localRegCtrl2);
  delay(measurePeriod);

  // Read the real acceleration.
  measures[3] = readAccelX();
  measures[4] = readAccelY();
  measures[5] = readAccelZ();

  // Reset default settings.
  writeToLSM303DRegister(LSM303D_CTRL2, regCtrl2);

  // Calculate the selftest difference.
  register float xdiff = 1000 * (measures[0] - measures[3]);
  register float ydiff = 1000 * (measures[1] - measures[4]);
  register float zdiff = 1000 * (measures[2] - measures[5]);

  // Output difference
  if(diff != NULL) {
    diff[0] = xdiff;
    diff[1] = ydiff;
    diff[2] = zdiff;
  }

  // Check if the sensor is working properly.
  return (xdiff >= 70
         && xdiff <= 1700
         && ydiff >= 70
         && ydiff <= 1700
         && zdiff >= 70
         && zdiff <= 1700) ||
         (xdiff <= -70     // Signal does not matter
         && xdiff >= -1700
         && ydiff <= -70
         && ydiff >= -1700
         && zdiff <= -70
         && zdiff >= -1700);
}

bool Nanoshield_IMU::accelHasNewData() {
  register int8_t statusa = readFromLSM303DRegister(LSM303D_STATUS_A);
  return statusa > 0;
}

float Nanoshield_IMU::readAccelX() {
  register int16_t xAccel = readFromLSM303DRegister(LSM303D_OUT_X_H_A) << 8;
  xAccel |= readFromLSM303DRegister(LSM303D_OUT_X_L_A);
  return (float) xAccel * accelScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelY() {
  register int16_t yAccel = readFromLSM303DRegister(LSM303D_OUT_Y_H_A) << 8;
  yAccel |= readFromLSM303DRegister(LSM303D_OUT_Y_L_A);
  return (float) yAccel * accelScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelZ() {
  register int16_t zAccel = readFromLSM303DRegister(LSM303D_OUT_Z_H_A) << 8;
  zAccel |= readFromLSM303DRegister(LSM303D_OUT_Z_L_A);
  return (float) zAccel * accelScale / INT16_T_TOP;
}

void Nanoshield_IMU::setMagnetometerPowerDown() {
  regCtrl7 |= LSM303D_MD_POWERDOWN;

  writeIfHasBegun(LSM303D_CTRL7, regCtrl7);
}

void Nanoshield_IMU::setMagnetometerDataRate(int8_t drate) {
  if((regCtrl7 & LSM303D_MD_MASK) != LSM303D_MD_CONTINUOUS) {
    setMagnetometerContinuousMode();
  }

  regCtrl5 &= ~LSM303D_M_ODR_MASK;
  regCtrl5 |= drate;

  writeIfHasBegun(LSM303D_CTRL5, regCtrl5);
}

void Nanoshield_IMU::setMagnetometerFullScale(int8_t scale) {
  switch(scale) {
    case LSM303D_MFS_2GAUSS:
      magnetScale = 2;
      break;
    case LSM303D_MFS_4GAUSS:
      magnetScale = 4;
      break;
    case LSM303D_MFS_8GAUSS:
      magnetScale = 8;
      break;
    case LSM303D_MFS_12GAUSS:
      magnetScale = 12;
      break;
    default:
      return;
  }

  regCtrl6 &= ~LSM303D_MFS_MASK;
  regCtrl6 |= scale;

  writeIfHasBegun(LSM303D_CTRL6, regCtrl6);
}

bool Nanoshield_IMU::magnetHasNewData(){
  register int8_t statusm = readFromLSM303DRegister(LSM303D_STATUS_M);
  return statusm > 0;
}

void Nanoshield_IMU::setMagnetometerContinuousMode() {
  regCtrl7 &= ~LSM303D_MD_MASK;
  regCtrl7 |= LSM303D_MD_CONTINUOUS;

  writeIfHasBegun(LSM303D_CTRL7, regCtrl7);
}

void Nanoshield_IMU::setMagnetometerSingleShot() {
  regCtrl7 &= ~LSM303D_MD_MASK;
  regCtrl7 |= LSM303D_MD_SINGLECONV;

  writeIfHasBegun(LSM303D_CTRL7, regCtrl7);
}

float Nanoshield_IMU::readMagnetX() {
  register int16_t xMagnet = readFromLSM303DRegister(LSM303D_OUT_X_H_M) << 8;
  xMagnet |= readFromLSM303DRegister(LSM303D_OUT_X_L_M);
  return (float) xMagnet * magnetScale / INT16_T_TOP;
}

float Nanoshield_IMU::readMagnetY() {
  register int16_t yMagnet = readFromLSM303DRegister(LSM303D_OUT_Y_H_M) << 8;
  yMagnet |= readFromLSM303DRegister(LSM303D_OUT_Y_L_M);
  return (float) yMagnet * magnetScale / INT16_T_TOP;
}

float Nanoshield_IMU::readMagnetZ() {
  register int16_t zMagnet = readFromLSM303DRegister(LSM303D_OUT_Z_H_M) << 8;
  zMagnet |= readFromLSM303DRegister(LSM303D_OUT_Z_L_M);
  return (float) zMagnet * magnetScale / INT16_T_TOP;
}

void Nanoshield_IMU::setInterrupt1Source(int8_t src) {
  writeToLSM303DRegister(LSM303D_CTRL3, src);
}

void Nanoshield_IMU::setInterrupt2Source(int8_t src) {

}

void Nanoshield_IMU::writeToLSM303DRegister(int8_t reg, int8_t value) {
  Wire.beginTransmission(lsm303dAddress);
  Wire.write(reg);
  Wire.write(value);
  i2cError = Wire.endTransmission();
}

int Nanoshield_IMU::i2cStatus() {
  return i2cError;
}

void Nanoshield_IMU::writeIfHasBegun(int8_t reg, int8_t value) {
  if(hasBegun) {
    writeToLSM303DRegister(reg, value);
  }
}

int Nanoshield_IMU::readFromLSM303DRegister(int8_t reg) {
  Wire.beginTransmission(lsm303dAddress);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(lsm303dAddress, 1);
  return Wire.read();
}