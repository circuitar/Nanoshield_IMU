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

bool Nanoshield_IMU::selfTest(float diff[]) {
  register int8_t localRegCtrl2 = regCtrl2;
  register float measurePeriod;
  float measures[6];

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
  delay(measurePeriod);

  localRegCtrl2 &= ~LSM303D_AFS_MASK;
  localRegCtrl2 |= LSM303D_AFS_2G;  // Sets to operate in +/- 2g.
  localRegCtrl2 |= LSM303D_AST;
  writeToAccelerometerRegister(LSM303D_CTRL2, localRegCtrl2);
  delay(measurePeriod);

  measures[0] = readAccelX();
  measures[1] = readAccelY();
  measures[2] = readAccelZ();

  localRegCtrl2 &= ~LSM303D_AST;
  writeToAccelerometerRegister(LSM303D_CTRL2, localRegCtrl2);
  delay(measurePeriod);

  measures[3] = readAccelX();
  measures[4] = readAccelY();
  measures[5] = readAccelZ();

  writeToAccelerometerRegister(LSM303D_CTRL2, regCtrl2);

  register float xdiff = 1000 * (measures[0] - measures[3]);
  register float ydiff = 1000 * (measures[1] - measures[4]);
  register float zdiff = 1000 * (measures[2] - measures[5]);

  if(diff != NULL) {
    diff[0] = xdiff;
    diff[1] = ydiff;
    diff[2] = zdiff;
  }

  return (xdiff >= 70
         && xdiff <= 1700
         && ydiff >= 70
         && ydiff <= 1700
         && zdiff >= 70
         && zdiff <= 1700) ||
         (xdiff <= -70
         && xdiff >= -1700
         && ydiff <= -70
         && ydiff >= -1700
         && zdiff <= -70
         && zdiff >= -1700);
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
  register int16_t xAccel = readFromAccelerometerRegister(LSM303D_OUT_X_H_A) << 8;
  xAccel |= readFromAccelerometerRegister(LSM303D_OUT_X_L_A);
  return (float) xAccel * actualScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelY() {
  register int16_t yAccel = readFromAccelerometerRegister(LSM303D_OUT_Y_H_A) << 8;
  yAccel |= readFromAccelerometerRegister(LSM303D_OUT_Y_L_A);
  return (float) yAccel * actualScale / INT16_T_TOP;
}

float Nanoshield_IMU::readAccelZ() {
  register int16_t zAccel = readFromAccelerometerRegister(LSM303D_OUT_Z_H_A) << 8;
  zAccel |= readFromAccelerometerRegister(LSM303D_OUT_Z_L_A);
  return (float) zAccel * actualScale / INT16_T_TOP;
}

void Nanoshield_IMU::writeToAccelerometerRegister(int8_t reg, int8_t value) {
  Wire.beginTransmission(accelAddress);
  Wire.write(reg);
  Wire.write(value);
  i2cError = Wire.endTransmission();
}

int Nanoshield_IMU::i2cStatus() {
  return i2cError;
}

void Nanoshield_IMU::writeIfHasBegun(int8_t reg, int8_t value) {
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