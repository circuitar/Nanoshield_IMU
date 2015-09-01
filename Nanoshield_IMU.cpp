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
  l3gd20hAddress = L3GD20H_ADDRESS | (addr & 0x01);

  hasBegun = false;
  accelScale = 2;
  magnetScale = 2;
  gyroScale = 245;

  regCtrl0 = 0;
  regCtrl1 = 0 | LSM303D_AODR_1600  // Accelerometer data rate 1600Hz.
               | LSM303D_BDU        // Waits a read operation ends before updates a output register.
               | LSM303D_AZEN       // Accelerometer Z axis enabled.
               | LSM303D_AYEN       // Accelerometer Y axis enabled.
               | LSM303D_AXEN;      // Accelerometer X axis enabled.
  regCtrl2 = 0 | LSM303D_ABW_773    // Accelerometer anti-alias filter bandwidth 773Hz.
               | LSM303D_AFS_2G;    // Accelerometer full-scale +/- 2g.
  regCtrl5 = 0 | LSM303D_MODR_100;  // Magnetic data rate 100Hz.
  regCtrl6 = 0 | LSM303D_MFS_2GAUSS;    // Magnetic full-scale +/- 2gauss.
  regCtrl7 = 0 | LSM303D_MD_CONTINUOUS; // Magnetometer in continuous mode.

  fifoCtrl = 0;                     // FIFO disabled.

  // Interrupt generators disabled.
  igCfg1 = 0;
  igThs1 = 0; 
  igDur1 = 0; 
  igCfg2 = 0; 
  igThs2 = 0; 
  igDur2 = 0;

  gyroCtrl1 = 0 | L3GD20H_DRBW_800_100  // Data rate: 800Hz. Anti-alias filter: 100Hz.
                | L3GD20H_POWER_UP      // Gyroscope on.
                | L3GD20H_Z_ENABLE      // Gyroscope Z axis enabled.
                | L3GD20H_Y_ENABLE      // Gyroscope Y axis enabled.
                | L3GD20H_X_ENABLE;     // Gyroscope X axis enabled.
  gyroCtrl3 = 0;                        // No interruptions set.
  gyroCtrl4 = 0 | L3GD20H_BDU           // Waits a read operation end before updates a output register.
                | L3GD20H_FS_245;       // 245 degrees/second full scale.
  gyroCtrl5 = 0;
}

void Nanoshield_IMU::begin() {
  Wire.begin();

  writeToRegister(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
  writeToRegister(lsm303dAddress, LSM303D_CTRL2, regCtrl2);
  writeToRegister(lsm303dAddress, LSM303D_CTRL5, regCtrl5);
  writeToRegister(lsm303dAddress, LSM303D_CTRL6, regCtrl6);
  writeToRegister(lsm303dAddress, LSM303D_CTRL7, regCtrl7);

  // Not initializing CTRL0 and FIFO_CTRL with 0 results in some problems with FIFO.
  writeToRegister(lsm303dAddress, LSM303D_CTRL0, 0);
  writeToRegister(lsm303dAddress, LSM303D_FIFO_CTRL, 0);

  // Just rewrite CTRL0 and FIFO_CTRL if they are set to something
  if(regCtrl0 != 0 || fifoCtrl != 0) {
    writeToRegister(lsm303dAddress, LSM303D_CTRL0, regCtrl0);
    writeToRegister(lsm303dAddress, LSM303D_FIFO_CTRL, fifoCtrl);
  }

  writeToRegister(lsm303dAddress, LSM303D_IG_CFG1, igCfg1);
  writeToRegister(lsm303dAddress, LSM303D_IG_THS1, igThs1);
  writeToRegister(lsm303dAddress, LSM303D_IG_DUR1, igDur1);
  writeToRegister(lsm303dAddress, LSM303D_IG_CFG2, igCfg2);
  writeToRegister(lsm303dAddress, LSM303D_IG_THS2, igThs2);
  writeToRegister(lsm303dAddress, LSM303D_IG_DUR2, igDur2);

  writeToRegister(l3gd20hAddress, L3GD20H_CTRL1, gyroCtrl1);
  writeToRegister(l3gd20hAddress, L3GD20H_CTRL3, gyroCtrl3);
  writeToRegister(l3gd20hAddress, L3GD20H_CTRL4, gyroCtrl4);
  writeToRegister(l3gd20hAddress, L3GD20H_CTRL5, gyroCtrl5);

  hasBegun = true;
}

void Nanoshield_IMU::setAccelerometerPowerDown() {
  regCtrl1 &= ~LSM303D_AODR_MASK;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::setAccelerometerDataRate(int8_t drate) {
  regCtrl1 &= ~LSM303D_AODR_MASK;
  regCtrl1 |= drate;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::enableAccelXAxis() {
  regCtrl1 |= LSM303D_AXEN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::disableAccelXAxis() {
  regCtrl1 &= ~LSM303D_AXEN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::enableAccelYAxis() {
  regCtrl1 |= LSM303D_AYEN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::disableAccelYAxis() {
  regCtrl1 &= ~LSM303D_AYEN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::enableAccelZAxis() {
  regCtrl1 |= LSM303D_AZEN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
}

void Nanoshield_IMU::disableAccelZAxis() {
  regCtrl1 &= ~LSM303D_AZEN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL1, regCtrl1);
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

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL2, regCtrl2);
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
  writeToRegister(lsm303dAddress, LSM303D_CTRL2, localRegCtrl2);
  delay(measurePeriod);

  // Read the simulated acceleration.
  measures[0] = readAccelX();
  measures[1] = readAccelY();
  measures[2] = readAccelZ();

  // Unset the self test flag
  // and wait next measure to be ready.
  localRegCtrl2 &= ~LSM303D_AST;
  writeToRegister(lsm303dAddress, LSM303D_CTRL2, localRegCtrl2);
  delay(measurePeriod);

  // Read the real acceleration.
  measures[3] = readAccelX();
  measures[4] = readAccelY();
  measures[5] = readAccelZ();

  // Reset default settings.
  writeToRegister(lsm303dAddress, LSM303D_CTRL2, regCtrl2);

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
         && zdiff <= 1700)
         || (xdiff <= -70     // Signal does not matter
         && xdiff >= -1700
         && ydiff <= -70
         && ydiff >= -1700
         && zdiff <= -70
         && zdiff >= -1700);
}

void Nanoshield_IMU::setAccelAntialiasFilter(int8_t bandwidth) {
  regCtrl2 &= ~LSM303D_ABW_MASK;
  regCtrl2 |= bandwidth;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL2, regCtrl2);
}

bool Nanoshield_IMU::accelHasNewData() {
  register int8_t statusa = readFromRegister(lsm303dAddress, LSM303D_STATUS_A);
  return statusa > 0;
}

float Nanoshield_IMU::readAccelX() {
  register int16_t xAccel = readFromRegister(lsm303dAddress, LSM303D_OUT_X_H_A) << 8;
  xAccel |= readFromRegister(lsm303dAddress, LSM303D_OUT_X_L_A);
  return (float) xAccel * accelScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelY() {
  register int16_t yAccel = readFromRegister(lsm303dAddress, LSM303D_OUT_Y_H_A) << 8;
  yAccel |= readFromRegister(lsm303dAddress, LSM303D_OUT_Y_L_A);
  return (float) yAccel * accelScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelZ() {
  register int16_t zAccel = readFromRegister(lsm303dAddress, LSM303D_OUT_Z_H_A) << 8;
  zAccel |= readFromRegister(lsm303dAddress, LSM303D_OUT_Z_L_A);
  return (float) zAccel * accelScale / INT16_T_TOP;
}

void Nanoshield_IMU::setMagnetometerPowerDown() {
  regCtrl7 |= LSM303D_MD_POWERDOWN;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL7, regCtrl7);
}

void Nanoshield_IMU::setMagnetometerDataRate(int8_t drate) {
  if((regCtrl7 & LSM303D_MD_MASK) != LSM303D_MD_CONTINUOUS) {
    setMagnetometerContinuousMode();
  }

  regCtrl5 &= ~LSM303D_MODR_MASK;
  regCtrl5 |= drate;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL5, regCtrl5);
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

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL6, regCtrl6);
}

bool Nanoshield_IMU::magnetHasNewData(){
  register int8_t statusm = readFromRegister(lsm303dAddress, LSM303D_STATUS_M);
  return statusm > 0;
}

void Nanoshield_IMU::setMagnetometerContinuousMode() {
  regCtrl7 &= ~LSM303D_MD_MASK;
  regCtrl7 |= LSM303D_MD_CONTINUOUS;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL7, regCtrl7);
}

void Nanoshield_IMU::setMagnetometerSingleShot() {
  regCtrl7 &= ~LSM303D_MD_MASK;
  regCtrl7 |= LSM303D_MD_SINGLECONV;

  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL7, regCtrl7);
}

float Nanoshield_IMU::readMagnetX() {
  register int16_t xMagnet = readFromRegister(lsm303dAddress, LSM303D_OUT_X_H_M) << 8;
  xMagnet |= readFromRegister(lsm303dAddress, LSM303D_OUT_X_L_M);
  return (float) xMagnet * magnetScale / INT16_T_TOP;
}

float Nanoshield_IMU::readMagnetY() {
  register int16_t yMagnet = readFromRegister(lsm303dAddress, LSM303D_OUT_Y_H_M) << 8;
  yMagnet |= readFromRegister(lsm303dAddress, LSM303D_OUT_Y_L_M);
  return (float) yMagnet * magnetScale / INT16_T_TOP;
}

float Nanoshield_IMU::readMagnetZ() {
  register int16_t zMagnet = readFromRegister(lsm303dAddress, LSM303D_OUT_Z_H_M) << 8;
  zMagnet |= readFromRegister(lsm303dAddress, LSM303D_OUT_Z_L_M);
  return (float) zMagnet * magnetScale / INT16_T_TOP;
}

void Nanoshield_IMU::setInterrupt1Source(int8_t src) {
  writeToRegister(lsm303dAddress, LSM303D_CTRL3, src);
  switch(src) {
    case LSM303D_INT1_DRDY_A: 
      readAccelX();
      break;
  }
}

void Nanoshield_IMU::setInterrupt2Source(int8_t src) {
  writeToRegister(lsm303dAddress, LSM303D_CTRL4, src);
  switch(src) {
    case LSM303D_INT2_DRDY_A: 
      readAccelX();
      break;
  }
}

void Nanoshield_IMU::setGyroInterruptSource(int8_t src) {
  // gyroCtrl3 &= 0x0F;
  // gyroCtrl3 |= src;

  writeIfHasBegun(l3gd20hAddress, L3GD20H_CTRL3, src);
  switch(src) {
    case L3GD20H_INT2_DRDY:
      readGyroX();
      break;
  }
}

void Nanoshield_IMU::enableAccelBuffer(int8_t mode, int8_t threshold) {
  if(threshold < 31) {
    regCtrl0 |= LSM303D_FTH_EN;
    fifoCtrl |= threshold & LSM303D_THRESHOLD_MASK;
  }

  regCtrl0 |= LSM303D_FIFO_EN;
  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL0, regCtrl0);

  fifoCtrl |= mode & LSM303D_FIFO_MODE_MASK;
  writeIfHasBegun(lsm303dAddress, LSM303D_FIFO_CTRL, fifoCtrl);
}

void Nanoshield_IMU::disableAccelBuffer() {
  regCtrl0 &= ~(LSM303D_FIFO_EN
                | LSM303D_FTH_EN);
  writeIfHasBegun(lsm303dAddress, LSM303D_CTRL0, regCtrl0);
}

int Nanoshield_IMU::getBufferCount() {
  return readFromRegister(lsm303dAddress, LSM303D_FIFO_SRC) & LSM303D_FSS_MASK;
}

void Nanoshield_IMU::resetAccelBuffer() {
  writeToRegister(lsm303dAddress, LSM303D_FIFO_CTRL, fifoCtrl 
                                            & ~LSM303D_FIFO_MODE_MASK 
                                            | LSM303D_BYPASS);
  writeToRegister(lsm303dAddress, LSM303D_FIFO_CTRL, fifoCtrl);
}

void Nanoshield_IMU::setAccelIntGenerator1Mode(int8_t mode) {
  igCfg1 &= ~LSM303D_INTMODE_MASK;
  igCfg1 |= mode & LSM303D_INTMODE_MASK;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_CFG1, igCfg1);
}

void Nanoshield_IMU::addToAccelIntGenerator1Zone(int8_t zone) {
  igCfg1 |= zone & LSM303D_ZONE_MASK;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_CFG1, igCfg1);
}

void Nanoshield_IMU::removeFromAccelIntGenerator1Zone(int8_t zone) {
  igCfg1 &= ~(zone & LSM303D_ZONE_MASK);

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_CFG1, igCfg1);
}

void Nanoshield_IMU::setAccelIntGenerator1Threshold(float threshold) {
  igThs1 = (int8_t) (threshold * INT16_T_TOP / accelScale) & 0x7F;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_THS1, igThs1);
}

void Nanoshield_IMU::setAccelIntGenerator1Duration(int8_t duration) {
  igDur1 = duration & 0x7F;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_DUR1, igDur1);
}

int8_t Nanoshield_IMU::getAccelIntGenerator1Source() {
  return readFromRegister(lsm303dAddress, LSM303D_IG_SRC1);
}

void Nanoshield_IMU::setAccelIntGenerator2Mode(int8_t mode) {
  igCfg2 &= ~LSM303D_INTMODE_MASK;
  igCfg2 |= mode & LSM303D_INTMODE_MASK;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_CFG2, igCfg2);
}

void Nanoshield_IMU::addToAccelIntGenerator2Zone(int8_t zone) {
  igCfg2 |= zone & LSM303D_ZONE_MASK;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_CFG2, igCfg2);
}

void Nanoshield_IMU::removeFromAccelIntGenerator2Zone(int8_t zone) {
  igCfg2 &= ~(zone & LSM303D_ZONE_MASK);

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_CFG2, igCfg2);
}

void Nanoshield_IMU::setAccelIntGenerator2Threshold(float threshold) {
  igThs2 = (int8_t) (threshold * INT16_T_TOP / accelScale) & 0x7F;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_THS2, igThs2);
}

void Nanoshield_IMU::setAccelIntGenerator2Duration(int8_t duration) {
  igDur2 = duration & 0x7F;

  writeIfHasBegun(lsm303dAddress, LSM303D_IG_DUR2, igDur2);
}

int8_t Nanoshield_IMU::getAccelIntGenerator2Source() {
  return readFromRegister(lsm303dAddress, LSM303D_IG_SRC2);
}

void Nanoshield_IMU::setGyroscopeFullScale(int8_t scale) {
  switch(scale) {
    case L3GD20H_FS_245:
      gyroScale = 245;
      break;
    case L3GD20H_FS_500:
      gyroScale = 500;
      break;
    case L3GD20H_FS_2000:
      gyroScale = 2000;
      break;
    default:
      return;
  }

  gyroCtrl4 &= ~L3GD20H_FS_MASK;
  gyroCtrl4 |= scale;

  writeIfHasBegun(l3gd20hAddress, L3GD20H_CTRL4, gyroCtrl4);
}

void Nanoshield_IMU::setGyroscopeDataRate(int16_t drate) {
  gyroCtrl1 &= ~L3GD20H_DRBW_MASK;
  gyroCtrl1 |= drate & L3GD20H_DRBW_MASK;

  writeIfHasBegun(l3gd20hAddress, L3GD20H_CTRL1, gyroCtrl1);
  writeIfHasBegun(l3gd20hAddress, L3GD20H_LOW_ODR, drate >> 8 & 0x01);
}

float Nanoshield_IMU::readGyroX() {
  register int16_t xGyro = readFromRegister(l3gd20hAddress, L3GD20H_OUT_X_H) << 8;
  xGyro |= readFromRegister(l3gd20hAddress, L3GD20H_OUT_X_L);
  return (float) xGyro * gyroScale / INT16_T_TOP;
}

float Nanoshield_IMU::readGyroY() {
  register int16_t xGyro = readFromRegister(l3gd20hAddress, L3GD20H_OUT_Y_H) << 8;
  xGyro |= readFromRegister(l3gd20hAddress, L3GD20H_OUT_Y_L);
  return (float) xGyro * gyroScale / INT16_T_TOP;
}

float Nanoshield_IMU::readGyroZ() {
  register int16_t xGyro = readFromRegister(l3gd20hAddress, L3GD20H_OUT_Z_H) << 8;
  xGyro |= readFromRegister(l3gd20hAddress, L3GD20H_OUT_Z_L);
  return (float) xGyro * gyroScale / INT16_T_TOP;
}


void Nanoshield_IMU::writeToRegister(int8_t addr, int8_t reg, int8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  i2cError = Wire.endTransmission();
}

int Nanoshield_IMU::i2cStatus() {
  return i2cError;
}

void Nanoshield_IMU::writeIfHasBegun(int8_t addr, int8_t reg, int8_t value) {
  if(hasBegun) {
    writeToRegister(addr, reg, value);
  }
}

int8_t Nanoshield_IMU::readFromRegister(int8_t addr, int8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, 1);
  return Wire.read();
}