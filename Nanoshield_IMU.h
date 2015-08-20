#ifndef NANOSHIELD_IMU_H
#define NANOSHIELD_IMU_H

#include "Arduino.h"
#include <Wire.h>

// I2C Addresses
#define LSM303D_ADDRESS         (0x1C)
#define LSM303D_I2C_0           (0x02)
#define LSM303D_I2C_1           (0x01)

// Registers Addresses
#define LSM303D_TEMP_OUT_L      (0x05)
#define LSM303D_TEMP_OUT_H      (0x06)
#define LSM303D_STATUS_M        (0x07)
#define LSM303D_OUT_X_L_M       (0x08)  /// Lower address to magnetic X axis output.
#define LSM303D_OUT_X_H_M       (0x09)  /// Higher address to magnetic X axis output.
#define LSM303D_OUT_Y_L_M       (0x0A)  /// Lower address to magnetic Y axis output.
#define LSM303D_OUT_Y_H_M       (0x0B)  /// Higher address to magnetic Y axis output.
#define LSM303D_OUT_Z_L_M       (0x0C)  /// Lower address to magnetic Z axis output.
#define LSM303D_OUT_Z_H_M       (0x0D)  /// Higher address to magnetic Z axis output.
#define LSM303D_WHO_AM_I        (0x0F)
#define LSM303D_INT_CTRL_M      (0x12)
#define LSM303D_INT_SRC_M       (0x13)
#define LSM303D_THS_L_M         (0x14)
#define LSM303D_THS_H_M         (0x15)
#define LSM303D_OFFSET_X_L_M    (0x16)  /// Lower address to magnetic offset for X axis.
#define LSM303D_OFFSET_X_H_M    (0x17)  /// Higher address to magnetic offset for X axis.
#define LSM303D_OFFSET_Y_L_M    (0x18)  /// Lower address to magnetic offset for Y axis.
#define LSM303D_OFFSET_Y_H_M    (0x19)  /// Higher address to magnetic offset for Y axis.
#define LSM303D_OFFSET_Z_L_M    (0x1A)  /// Lower address to magnetic offset for Z axis.
#define LSM303D_OFFSET_Z_H_M    (0x1B)  /// Higher address to magnetic offset for Z axis.
#define LSM303D_REFERENCE_X     (0x1C)  /// Reference value for high-pass filter for X-axis acceleration data.
#define LSM303D_REFERENCE_Y     (0x1D)  /// Reference value for high-pass filter for Y-axis acceleration data.
#define LSM303D_REFERENCE_Z     (0x1E)  /// Reference value for high-pass filter for Z-axis acceleration data.
#define LSM303D_CTRL0           (0x1F)  /// Control register 0 address.
#define LSM303D_CTRL1           (0x20)  /// Control register 1 address.
#define LSM303D_CTRL2           (0x21)  /// Control register 2 address.
#define LSM303D_CTRL3           (0x22)  /// Control register 3 address.
#define LSM303D_CTRL4           (0x23)  /// Control register 4 address.
#define LSM303D_CTRL5           (0x24)  /// Control register 5 address.
#define LSM303D_CTRL6           (0x25)  /// Control register 6 address.
#define LSM303D_CTRL7           (0x26)  /// Control register 7 address.
#define LSM303D_STATUS_A        (0x27)
#define LSM303D_OUT_X_L_A       (0x28)  /// Lower address to magnetic X axis output.
#define LSM303D_OUT_X_H_A       (0x29)  /// Higher address to magnetic X axis output.
#define LSM303D_OUT_Y_L_A       (0x2A)  /// Lower address to magnetic Y axis output.
#define LSM303D_OUT_Y_H_A       (0x2B)  /// Higher address to magnetic Y axis output.
#define LSM303D_OUT_Z_L_A       (0x2C)  /// Lower address to magnetic Z axis output.
#define LSM303D_OUT_Z_H_A       (0x2D)  /// Higher address to magnetic Z axis output.
#define LSM303D_FIFO_CTRL       (0x2E)
#define LSM303D_FIFO_SRC        (0x2F)
#define LSM303D_IG_CFG1         (0x30)
#define LSM303D_IG_SRC1         (0x31)
#define LSM303D_IG_THS1         (0x32)
#define LSM303D_IG_DUR1         (0x33)
#define LSM303D_IG_CFG2         (0x34)
#define LSM303D_IG_SRC2         (0x35)
#define LSM303D_IG_THS2         (0x36)
#define LSM303D_IG_DUR2         (0x37)
#define LSM303D_CLICK_CFG       (0x38)
#define LSM303D_CLICK_SRC       (0x39)
#define LSM303D_CLICK_THS       (0x3A)
#define LSM303D_TIME_LIMIT      (0x3B)
#define LSM303D_TIME_LATENCY    (0x3C)
#define LSM303D_TIME_WINDOW     (0x3D)
#define LSM303D_ACT_THS         (0x3E)
#define LSM303D_ACT_DUR         (0x3F)

// LSM303D_CTRL0 Values
#define LSM303D_BOOT_RBT_MEM    (0x80)  /// Reboot memory content to restore default calibration parameters.
#define LSM303D_FIFO_EN         (0x40)  /// FIFO enable.
#define LSM303D_FTH_EN          (0x20)  /// FIFO programmable threshold enable.
#define LSM303D_HP_CLICK        (0x04)  /// High-pass filter enable for click function.
#define LSM303D_HPIS1           (0x02)  /// High-pass filter enable for interrupt generator 1.
#define LSM303D_HPIS2           (0x01)  /// High-pass filter enable for interrupt generator 2.

// LSM303D_CTRL1 Values
#define LSM303D_AODR_MASK       (0xF0)  /// Used to set acceleration power mode and data rate.
#define LSM303D_AODR_POWER_DOWN (0x00)  /// Accelerometer in power-down mode.
#define LSM303D_AODR_3_125      (0x10)  /// Accelerometer data rate 3.125Hz
#define LSM303D_AODR_6_25       (0x20)  /// Accelerometer data rate 6.25Hz
#define LSM303D_AODR_12_5       (0x30)  /// Accelerometer data rate 12.5Hz
#define LSM303D_AODR_25         (0x40)  /// Accelerometer data rate 25Hz
#define LSM303D_AODR_50         (0x50)  /// Accelerometer data rate 50Hz
#define LSM303D_AODR_100        (0x60)  /// Accelerometer data rate 100Hz
#define LSM303D_AODR_200        (0x70)  /// Accelerometer data rate 200Hz
#define LSM303D_AODR_400        (0x80)  /// Accelerometer data rate 400Hz
#define LSM303D_AODR_800        (0x90)  /// Accelerometer data rate 800Hz
#define LSM303D_AODR_1600       (0xA0)  /// Accelerometer data rate 1600Hz
#define LSM303D_BDU             (0x08)  /// Block data update: wait a read operation ends before update output register.
#define LSM303D_AZEN            (0x04)  /// Enable accelerometer Z axis
#define LSM303D_AYEN            (0x02)  /// Enable accelerometer Y axis.
#define LSM303D_AXEN            (0x01)  /// Enable accelerometer X axis.

// LSM303D_CTRL2 Values
#define LSM303D_ABW_MASK        (0xC0)  /// Accelerometer anti-alias filter bandwidth.
#define LSM303D_ABW_773         (0x00)  /// Accelerometer anti-alias filter bandwidth 773Hz.
#define LSM303D_ABW_194         (0x40)  /// Accelerometer anti-alias filter bandwidth 194Hz.
#define LSM303D_ABW_362         (0x80)  /// Accelerometer anti-alias filter bandwidth 362Hz.
#define LSM303D_ABW_50          (0xC0)  /// Accelerometer anti-alias filter bandwidth 50Hz.
#define LSM303D_AFS_MASK        (0x38)  /// Acceleration full-scale selection.
#define LSM303D_AFS_2G          (0x00)  /// Full-scale +/- 2g.
#define LSM303D_AFS_4G          (0x08)  /// Full-scale +/- 4g.
#define LSM303D_AFS_6G          (0x10)  /// Full-scale +/- 6g.
#define LSM303D_AFS_8G          (0x18)  /// Full-scale +/- 8g.
#define LSM303D_AFS_16G         (0x20)  /// Full-scale +/- 16g.
#define LSM303D_AST             (0x02)  /// Acceleration self-test enable.

// LSM303D_CTRL3 Values
#define LSM303D_INT1_BOOT       (0x80)  /// Boot on INT1 enable.
#define LSM303D_INT1_CLICK      (0x40)  /// Click generator interrupt on INT1.
#define LSM303D_INT1_IG1        (0x20)  /// Inertial interrupt generator 1 on INT1.
#define LSM303D_INT1_IG2        (0x10)  /// Inertial interrupt generator 2 on INT1.
#define LSM303D_INT1_IGM        (0x08)  /// Magnetic interrupt generator on INT1.
#define LSM303D_INT1_DRDY_A     (0x04)  /// Accelerometer data-ready signal on INT1.
#define LSM303D_INT1_DRDY_M     (0x02)  /// Magnetometer data-ready signal on INT1.
#define LSM303D_INT1_EMPTY      (0x01)  /// FIFO empty indication on INT1.

// LSM303D_CTRL4 Values
#define LSM303D_INT2_CLICK      (0x80)  /// Click generator interrupt on INT2.
#define LSM303D_INT2_IG1        (0x40)  /// Inertial interrupt generator 1 on INT2.
#define LSM303D_INT2_IG2        (0x20)  /// Inertial interrupt generator 2 on INT2.
#define LSM303D_INT2_IGM        (0x10)  /// Magnetic interrupt generator on INT2.
#define LSM303D_INT2_DRDY_A     (0x08)  /// Accelerometer data-ready signal on INT2.
#define LSM303D_INT2_DRDY_M     (0x04)  /// Magnetometer data-ready signal on INT2.
#define LSM303D_INT2_OVERRUN    (0x02)  /// FIFO overrun interrupt on INT2.
#define LSM303D_INT2_FTH        (0x01)  /// FIFO threshold interrupt on INT2.

// LSM303D_CTRL5 Values
#define LSM303D_TEMP_EN         (0x80)  /// Temperature sensor enable.
#define LSM303D_M_RES_MASK      (0x60)  /// Magnetic resolution selector.
#define LSM303D_M_RES_LOW       (0x00)  /// Magnetic low resolution.
#define LSM303D_M_RES_HIGH      (0x60)  /// Magnetic high resolution.
#define LSM303D_M_ODR_MASK      (0x1C)  /// Magnetic data rate selection.
#define LSM303D_M_ODR_3_125     (0x00)  /// Magnetic data rate 3.125Hz.
#define LSM303D_M_ODR_6_25      (0x04)  /// Magnetic data rate 6.25Hz.
#define LSM303D_M_ODR_12_5      (0x08)  /// Magnetic data rate 12.5Hz.
#define LSM303D_M_ODR_25        (0x0C)  /// Magnetic data rate 25Hz.
#define LSM303D_M_ODR_50        (0x10)  /// Magnetic data rate 50Hz.
#define LSM303D_M_ODR_100       (0x14)  /// Magnetic data rate 100Hz. Available only for accelerometer ODR > 50Hz or accelerometer in power-down mode.
#define LSM303D_LIR2            (0x02)  /// Latch interrupt request on LSM303D_IG_SRC2 register, with LSM303D_IG_SRC2 cleared by reading LSM303D_IG_SRC2 itself.
#define LSM303D_LIR1            (0x01)  /// Latch interrupt request on LSM303D_IG_SRC1 register, with LSM303D_IG_SRC1 cleared by reading LSM303D_IG_SRC1 itself.

// LSM303D_CTRL6 Values
#define LSM303D_MFS_MASK        (0x60)  /// Magnetic full-scale selection.
#define LSM303D_MFS_2GAUSS      (0x00)  /// Magnetic full-scale +/- 2 gauss.
#define LSM303D_MFS_4GAUSS      (0x20)  /// Magnetic full-scale +/- 4 gauss.
#define LSM303D_MFS_8GAUSS      (0x40)  /// Magnetic full-scale +/- 8 gauss.
#define LSM303D_MFS_12GAUSS     (0x60)  /// Magnetic full-scale +/- 12 gauss.

// LSM303D_CTRL7 Values
#define LSM303D_AHPM_MASK       (0xC0)  /// High-pass filter mode selection for acceleration data.
#define LSM303D_AHPM_NRML_MODE  (0x00)  /// Normal mode (reset X, Y and Z axes reading REFERENCE_X, REFERENCE_Y and REFERENCE_Z registers).
#define LSM303D_AHPM_REFERENCE  (0x40)  /// Reference signal for filtering.
#define LSM303D_AHPM_AUTORESET  (0xC0)  /// Auto-reset on interrupt event.
#define LSM303D_AFDS            (0x20)  /// Send filtered acceleration data to output register and FIFO.
#define LSM303D_T_ONLY          (0x10)  /// Temperature sensor only. Temperature sensor is on while magnetic sensor is off.
#define LSM303D_MLP             (0x04)  /// Magnetic data low-power mode. Magnetic output data rate is set to 3.125Hz independently of LSM303D_M_ODR value.
#define LSM303D_MD_MASK         (0x03)  /// Magnetic sensor mode selection.
#define LSM303D_MD_CONTINUOUS   (0x00)  /// Magnetic continuous conversion mode.
#define LSM303D_MD_SINGLECONV   (0x01)  /// Magnetic single conversion mode.
#define LSM303D_MD_POWERDOWN    (0x03)  /// Magnetic power-down mode.

class Nanoshield_IMU {
public:
  /**
   * @brief Constructor.
   * 
   * Creates an object to access the Nanoshield IMU.
   * To nanoshield address use LSM303D_I2C_1 or LSM303D_I2C_0, according to 
   * ADDR jump.
   * 
   * @param addr I2C address selected on hardware.
   */
  Nanoshield_IMU(int addr = LSM303D_I2C_1);

  /**
   * @brief Turns off the accelerometer.
   * 
   * Note that the last accelerometer measure stays in the registers. In case
   * of reading any axis, the returned value will be wrong. To turn the 
   * accelerometer on again, set a data rate to it.
   * 
   * @see setAccelerometerDataRate()
   */
  void setAccelerometerPowerDown();

  /**
   * @brief Sets accelerometer data rate.
   * 
   * Possible data rates:
   * - LSM303D_AODR_3_125: 3.125 Hz.
   * - LSM303D_AODR_6_25: 6.25 Hz.
   * - LSM303D_AODR_12_5: 12.5 Hz.
   * - LSM303D_AODR_25: 25 Hz.
   * - LSM303D_AODR_50: 50 Hz.
   * - LSM303D_AODR_100: 100 Hz.
   * - LSM303D_AODR_200: 200 Hz.
   * - LSM303D_AODR_400: 400 Hz.
   * - LSM303D_AODR_800: 800 Hz.
   * - LSM303D_AODR_1600: 1600 Hz.
   * Note that as greater is the data rate, greater is the power consumption
   * and as lower is the data rate, lower is the power consumption.
   * 
   * @param drate Frequency to refresh accelerometer measures.
   */
  void setAccelerometerDataRate(int8_t drate);

  /**
   * @brief Enable the accelerometer X axis.
   */
  void enableAccelXAxis();

  /**
   * @brief Disable the accelerometer X axis.
   * 
   * When the axis is disabled, the output register is set to zero.
   * To enable the axis again use enableAccelXAxis().
   * 
   * @see enableAccelXAxis()
   */
  void disableAccelXAxis();

  /**
   * @brief Enable the accelerometer Y axis.
   */
  void enableAccelYAxis();

  /**
   * @brief Disable the accelerometer Y axis.
   * 
   * When the axis is disabled, the output register is set to zero.
   * To enable the axis again use enableAccelYAxis().
   * 
   * @see enableAccelYAxis()
   */
  void disableAccelYAxis();

  /**
   * @brief Enable the accelerometer Z axis.
   */
  void enableAccelZAxis();

  /**
   * @brief Disable the accelerometer Z axis.
   * 
   * When the axis is disabled, the output register is set to zero.
   * To enable the axis again use enableAccelZAxis().
   * 
   * @see enableAccelZAxis()
   */
  void disableAccelZAxis();

  /**
   * @brief Sets the scale range.
   * 
   * Possible values:
   * - LSM303D_AFS_2G: +/- 2g (0.061 mg/LSB of sensity)
   * - LSM303D_AFS_4G: +/- 4g (0.122 mg/LSB of sensity)
   * - LSM303D_AFS_6G: +/- 6g (0.183 mg/LSB of sensity)
   * - LSM303D_AFS_8G: +/- 8g (0.244 mg/LSB of sensity)
   * - LSM303D_AFS_16G: +/- 16g (0.732 mg/LSB of sensity)
   * 
   * @param scale The scale to adjust the measure.
   */
  void setAccelerometerFullScale(int8_t scale);

  /**
   * @brief Checks if the sensor is working properly.
   * 
   * The LSM303D has a self test feature where an actuation force is applied to
   * the sensor, simulating a definity input acceleration. The LSM303D datasheet
   * sets the reference range to the difference between the simulated measure
   * and the real measure. This test must be done with accelerations applied
   * to the sensor, except the gravity.
   * 
   * @param diff Optional. A three sized float array to store the difference
   *             measured in X [0], Y [1] and Z [2].
   * @return True if the test is successful. False if the test fails.
   */
  bool selfTest(float diff[] = NULL);

  /**
   * @brief Initializes the communication with Nanoshield_IMU.
   */
  void begin();

  /**
   * @brief Gets the last acceleration measured on X axis.
   * 
   * @return The acceleration in g unit.
   */
  float readAccelX();

  /**
   * @brief Gets the last acceleration measured on Y axis.
   * 
   * @return The acceleration in g unit.
   */
  float readAccelY();

  /**
   * @brief Gets the last acceleration measured on Z axis.
   * 
   * @return The acceleration in g unit.
   */
  float readAccelZ();

  /**
   * @brief Turns off the magnetometer.
   * 
   * Note that the last magnetometer measure stays in the registers. In case
   * of reading any axis, the returned value will be wrong. To turn the 
   * magnetometer on again, set a data rate to it.
   * 
   * @see setMagnetometerDataRate()
   */
  void setMagnetometerPowerDown();

  /**
   * @brief Sets the magnetometer data rate.
   * 
   * Possible data rates:
   * - LSM303D_M_ODR_3_125: 3.125Hz.
   * - LSM303D_M_ODR_6_25: 6.25 Hz.
   * - LSM303D_M_ODR_12_5: 12.5 Hz.
   * - LSM303D_M_ODR_25: 25 Hz.
   * - LSM303D_M_ODR_50: 50 Hz.
   * - LSM303D_M_ODR_100: 100 Hz.
   * The 100Hz can be used only if accelerometer data rate is greater than 50Hz
   * or if accelerometer is in power down mode. Note that as greater is the
   * data rate, greater is the power consumption and as lower is the data rate,
   * lower is the power consumption.
   * 
   * @param drate Frequency to refresh magnetometer measures.
   */
  void setMagnetometerDataRate(int8_t drate);

  /**
   * @brief Sets the magnetometer scale range.
   * 
   * Possible values:
   * - LSM303D_MFS_2GAUSS: +/- 2gauss (0.080 mgauss/LSB of sensity)
   * - LSM303D_MFS_4GAUSS: +/- 4gauss (0.160 mgauss/LSB of sensity)
   * - LSM303D_MFS_8GAUSS: +/- 8gauss (0.320 mgauss/LSB of sensity)
   * - LSM303D_MFS_12GAUSS: +/- 12gauss (0.479 mgauss/LSB of sensity)
   * 
   * @param scale The scale to adjust the measure.
   */
  void setMagnetometerFulScale(int8_t scale);

  /**
   * @brief Gets the last magnetic field measured on X axis.
   * 
   * @return The magnetic field in gauss unit.
   */
  float readMagnetX();

  /**
   * @brief Gets the last magnetic field measured on Y axis.
   * 
   * @return The magnetic field in gauss unit.
   */
  float readMagnetY();

  /**
   * @brief Gets the last magnetic field measured on Z axis.
   * 
   * @return The magnetic field in gauss unit.
   */
  float readMagnetZ();

  /**
   * @brief Writes a byte to any accelerometer register.
   * 
   * @param reg Register address.
   * @param value Value to write.
   */
  void writeToLSM303DRegister(int8_t reg, int8_t value);

  /**
   * @brief Reads 16bits from any accelerometer register.
   * 
   * @param reg Register to read.
   * @return 16bits value read.
   */
  int16_t readFromLSM303DRegister(int8_t reg);

  /**
   * @brief Checks the status of the last I2C communication with the Nanoshield.
   * 
   * @return The status returned by the last Wire.endTransmission().
   */
  int i2cStatus();

protected:
  int8_t lsm303dAddress;

  int8_t regCtrl0;
  int8_t regCtrl1;
  int8_t regCtrl2;
  int8_t regCtrl3;
  int8_t regCtrl4;
  int8_t regCtrl5;
  int8_t regCtrl6;
  int8_t regCtrl7;

  int i2cError;
  int8_t accelScale;
  int8_t magnetScale;
  bool hasBegun;

  void writeIfHasBegun(int8_t reg, int8_t value);
};


#endif