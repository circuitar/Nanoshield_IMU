/**
 * @file Nanoshield_IMU.h
 * 
 * A library to access the Nanoshield IMU v2.0.
 * It uses the LSM303D for accelerometer, magnetometer and temperature.
 * 
 * Copyright (c) 2015 Circuitar
 * This software is released under the MIT license. See the attached LICENSE file for details.
 */
#ifndef NANOSHIELD_IMU_H
#define NANOSHIELD_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// I2C Addresses
#define LSM303D_ADDRESS         (0x1C)
#define L3GD20H_ADDRESS         (0x6A)
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
#define LSM303D_STATUS_A        (0x27)  /// Status about accelerometer new data or data overrun.
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

// LSM303D_STATUS_M Values
#define LSM303D_ZYXMOR          (0x80)
#define LSM303D_ZMOR            (0x40)  /// Magnetometer Z axis data overrun.
#define LSM303D_YMOR            (0x20)  /// Magnetometer Y axis data overrun.
#define LSM303D_XMOR            (0x10)  /// Magnetometer X axis data overrun.
#define LSM303D_ZYXMDA          (0x08)
#define LSM303D_ZMDA            (0x04)  /// Magnetometer Z axis new data.
#define LSM303D_YMDA            (0x02)  /// Magnetometer Y axis new data.
#define LSM303D_XMDA            (0x01)  /// Magnetometer X axis new data.

// LSM303D_INT_CTRL_M Values
#define LSM303D_XMIEN           (0x80)  /// Enable interrupt recognition on X axis for magnetic data.
#define LSM303D_YMIEN           (0x40)  /// Enable interrupt recognition on Y axis for magnetic data.
#define LSM303D_ZMIEN           (0x20)  /// Enable interrupt recognition on Z axis for magnetic data.
#define LSM303D_PP_OD           (0x10)  /// Interrupt signal generated with push-pull circuit (0) or open-drain circuit(1).
#define LSM303D_MIEA            (0x08)  /// Interrupt polarity low (0) or high (1).
#define LSM303D_MIEL            (0x04)  /// Latch interrupt request. The interrupt is cleared by reading LSM303D_INT_SRC_M.
#define LSM303D_MIEN            (0x01)  /// Enable interrupt generation for magnetic data.

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
#define LSM303D_MODR_MASK       (0x1C)  /// Magnetic data rate selection.
#define LSM303D_MODR_3_125      (0x00)  /// Magnetic data rate 3.125Hz.
#define LSM303D_MODR_6_25       (0x04)  /// Magnetic data rate 6.25Hz.
#define LSM303D_MODR_12_5       (0x08)  /// Magnetic data rate 12.5Hz.
#define LSM303D_MODR_25         (0x0C)  /// Magnetic data rate 25Hz.
#define LSM303D_MODR_50         (0x10)  /// Magnetic data rate 50Hz.
#define LSM303D_MODR_100        (0x14)  /// Magnetic data rate 100Hz. Available only for accelerometer ODR > 50Hz or accelerometer in power-down mode.
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

// LSM303D_STATUS_A Values
#define LSM303D_ZYXAOR          (0x80)
#define LSM303D_ZAOR            (0x40)  /// Accelerometer Z axis data overrun.
#define LSM303D_YAOR            (0x20)  /// Accelerometer Y axis data overrun.
#define LSM303D_XAOR            (0x10)  /// Accelerometer X axis data overrun.
#define LSM303D_ZYXADA          (0x08)
#define LSM303D_ZADA            (0x04)  /// Accelerometer Z axis new data.
#define LSM303D_YADA            (0x02)  /// Accelerometer Y axis new data.
#define LSM303D_XADA            (0x01)  /// Accelerometer X axis new data.

// LSM303D_FIFO_CTRL Values
#define LSM303D_FIFO_MODE_MASK  (0xE0)
#define LSM303D_BYPASS          (0x00)
#define LSM303D_FIFO            (0x20)
#define LSM303D_STREAM          (0x40)
#define LSM303D_STREAM2FIFO     (0x60)
#define LSM303D_BYPASS2STREAM   (0x80)
#define LSM303D_THRESHOLD_MASK  (0x1F)

// LSM303D_FIFO_SRC Values
#define LSM303D_FTH_STATUS      (0x80)  /// Notify a threshold hit.
#define LSM303D_OVRN_STATUS     (0x40)  /// Notify a buffer overrun
#define LSM303D_EMPTY_STATUS    (0x20)  /// Notify empty buffer
#define LSM303D_FSS_MASK        (0x1F)

// LSM303D_IG_CFG1/LSM303D_IG_CFG2 Values
#define LSM303D_INTMODE_MASK      (0xC0)
#define LSM303D_INTMODE_OR        (0x00)  /// Triggers interruption on or combination.
#define LSM303D_INTMODE_MOVEMENT  (0x40)  /// Pulses interruption when moves to a known zone.
#define LSM303D_INTMODE_AND       (0x80)  /// Triggers interruption on and combination.
#define LSM303D_INTMODE_POSITION  (0xC0)  /// Keeps interruption signal while inside a known zone.
#define LSM303D_ZONE_MASK         (0x3F)
#define LSM303D_ZONE_ZH           (0x20)  /// Enable interrupt on high Z event.
#define LSM303D_ZONE_ZL           (0x10)  /// Enable interrupt on low Z event.
#define LSM303D_ZONE_YH           (0x08)  /// Enable interrupt on high Y event.
#define LSM303D_ZONE_YL           (0x04)  /// Enable interrupt on low Y event.
#define LSM303D_ZONE_XH           (0x02)  /// Enable interrupt on high X event.
#define LSM303D_ZONE_XL           (0x01)  /// Enable interrupt on low X event.

// LSM303D_IG_SRC1/LSM303D_IG_SRC2 Values
#define LSM303D_INT_ACTIVE    (0x40)  /// Notify that an IG interruption occurred. Reading the LSM303D_IG_SRC1/LSM303D_IG_SRC2 will clear the respective bit.
/*
 * To check the axes the zone flags can be used:
 * LSM303D_ZONE_MASK
 * LSM303D_ZONE_ZH
 * LSM303D_ZONE_ZL
 * LSM303D_ZONE_YH
 * LSM303D_ZONE_YL
 * LSM303D_ZONE_XH
 * LSM303D_ZONE_XL
 */

#define L3GD20H_CTRL1         (0x20)
#define L3GD20H_CTRL2         (0x21)
#define L3GD20H_CTRL3         (0x22)
#define L3GD20H_CTRL4         (0x23)
#define L3GD20H_CTRL5         (0x24)
#define L3GD20H_REFERENCE     (0x25)
#define L3GD20H_OUT_TEMP      (0x26)
#define L3GD20H_STATUS        (0x27)
#define L3GD20H_OUT_X_L       (0x28)
#define L3GD20H_OUT_X_H       (0x29)
#define L3GD20H_OUT_Y_L       (0x2A)
#define L3GD20H_OUT_Y_H       (0x2B)
#define L3GD20H_OUT_Z_L       (0x2C)
#define L3GD20H_OUT_Z_H       (0x2D)
#define L3GD20H_FIFO_CTRL     (0x2E)
#define L3GD20H_FIFO_SRC      (0x2F)
#define L3GD20H_IG_CFG        (0x30)
#define L3GD20H_IG_SRC        (0x31)
#define L3GD20H_IG_THS_XH     (0x32)
#define L3GD20H_IG_THS_XL     (0x33)
#define L3GD20H_IG_THS_YH     (0x34)
#define L3GD20H_IG_THS_YL     (0x35)
#define L3GD20H_IG_THS_ZH     (0x36)
#define L3GD20H_IG_THS_ZL     (0x37)
#define L3GD20H_IG_DURATION   (0x38)
#define L3GD20H_LOW_ODR       (0x39)

// L3GD20H_CTRL1 Values
#define L3GD20H_DRBW_MASK     (0xC0)
#define L3GD20H_DRBW_12_5     (0x100)
#define L3GD20H_DRBW_25       (0x140)
#define L3GD20H_DRBW_50_16_6  (0x180)
#define L3GD20H_DRBW_100_12_5 (0x00)
#define L3GD20H_DRBW_100_25   (0x10)
#define L3GD20H_DRBW_200_12_5 (0x40)
#define L3GD20H_DRBW_200_70   (0x70)
#define L3GD20H_DRBW_400_20   (0x80)
#define L3GD20H_DRBW_400_25   (0x90)
#define L3GD20H_DRBW_400_50   (0xA0)
#define L3GD20H_DRBW_400_110  (0xB0)
#define L3GD20H_DRBW_800_30   (0xC0)
#define L3GD20H_DRBW_800_35   (0xE0)
#define L3GD20H_DRBW_800_100  (0xF0)
#define L3GD20H_POWER_UP      (0x08)
#define L3GD20H_Z_ENABLE      (0x04)
#define L3GD20H_Y_ENABLE      (0x02)
#define L3GD20H_X_ENABLE      (0x01)

// L3GD20H_CTRL2 Values
#define L3GD20H_EXTREN        (0x80)
#define L3GD20H_LVLEN         (0x40)
#define L3GD20H_HPM_NORMAL    (0x00)
#define L3GD20H_HPM_REFERENCE (0x10)
#define L3GD20H_HPM_AUTORESET (0x30)
#define L3GD20H_HPM_CUTOFF_0  (0x00)
#define L3GD20H_HPM_CUTOFF_1  (0x01)
#define L3GD20H_CUTOFF_2      (0x02)
#define L3GD20H_CUTOFF_3      (0x03)
#define L3GD20H_CUTOFF_4      (0x04)
#define L3GD20H_CUTOFF_5      (0x05)
#define L3GD20H_CUTOFF_6      (0x06)
#define L3GD20H_CUTOFF_7      (0x07)
#define L3GD20H_CUTOFF_8      (0x08)
#define L3GD20H_CUTOFF_9      (0x09)

// L3GD20H_CTRL3 Values
#define L3GD20H_INT1_IG       (0x80)
#define L3GD20H_INT1_BOOT     (0x40)
#define L3GD20H_INT_HIGH_LOW  (0x20)
#define L3GD20H_INT_PP_OP     (0x10)
#define L3GD20H_INT2_DRDY     (0x08)
#define L3GD20H_INT2_FTH      (0x04)
#define L3GD20H_INT2_OVR      (0x02)
#define L3GD20H_INT2_EMPTY    (0x01)

// L3GD20H_CTRL4 Values
#define L3GD20H_BDU           (0x80)
#define L3GD20H_BLE           (0x40)
#define L3GD20H_FS_MASK       (0x30)
#define L3GD20H_FS_245        (0x00)
#define L3GD20H_FS_500        (0x10)
#define L3GD20H_FS_2000       (0x20)
#define L3GD20H_LVL_LATCHED   (0x08)
#define L3GD20H_ST_MASK       (0x06)
#define L3GD20H_ST_NORMAL     (0x00)
#define L3GD20H_ST_0          (0x02)
#define L3GD20H_ST_1          (0x06)

// L3GD20H_CTRL5 Values
#define L3GD20H_BOOT          (0x80)
#define L3GD20H_FIFO_EN       (0x40)
#define L3GD20H_STOP_ON_FTH   (0x20)
#define L3GD20H_HPEN          (0x10)

// L3GD20H_FIFO_CTRL Values
#define L3GD20H_FM_MASK         (0xE0)
#define L3GD20H_FM_BYPASS       (0x00)
#define L3GD20H_FM_FIFO         (0x20)
#define L3GD20H_FM_STREAM       (0x40)
#define L3GD20H_FM_STREAM2FIFO  (0x60)
#define L3GD20H_FM_BYPASS2STRM  (0x80)
#define L3GD20H_FM_DYNAMICSTRM  (0xC0)
#define L3GD20H_FM_BYPASS2FIFO  (0xE0)
#define L3GD20H_FTHS_MASK       (0x1F)

// L3GD20H_FIFO_SRC Values
#define L3GD20H_FTH             (0x80)
#define L3GD20H_OVRN            (0x40)
#define L3GD20H_EMPTY           (0x20)
#define L3GD20H_FSS_MASK        (0x1F)

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
   * @brief Initializes the communication with Nanoshield_IMU.
   */
  void begin();

  /**
   * @brief Turns off the accelerometer.
   * 
   * Note that the last accelerometer measure stays in the registers. In case
   * of reading any axes, the returned value will be wrong. To turn the 
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
   * @brief Filter the analog input signal before sampling the data.
   *
   * The anti-alias filter restricts the bandwidth of a signal to satisfy the 
   * Nyquist sampling theorem.
   * 
   * Possible values to bandwisth is
   * - LSM303D_ABW_773: 773Hz.
   * - LSM303D_ABW_362: 362Hz.
   * - LSM303D_ABW_194: 194Hz.
   * - LSM303D_ABW_50: 50Hz.
   * 
   * @param bandwidth The bandwidth to filter.
   */
  void setAccelAntialiasFilter(int8_t bandwidth);

  /**
   * @brief Checks for new accelerometer data.
   * 
   * @return True if a new value is available. False otherwise.
   */
  bool accelHasNewData();

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
   * @brief Gets the raw accelerometer measured on X axis.
   * 
   * To use a measure in g unit, use readAccelX().
   * 
   * @return 16 bit integer measured from CI.
   * 
   * @see readAccelX()
   */
  int16_t accelRawX();

  /**
   * @brief Gets the raw accelerometer measured on Y axis.
   * 
   * To use a measure in g unit, use readAccelY().
   * 
   * @return 16 bit integer measured from CI.
   * 
   * @see readAccelY()
   */
  int16_t accelRawY();

  /**
   * @brief Gets the raw accelerometer measured on Z axis.
   * 
   * To use a measure in g unit, use readAccelZ().
   * 
   * @return 16 bit integer measured from CI.
   * 
   * @see readAccelZ()
   */
  int16_t accelRawZ();

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
  void setMagnetometerFullScale(int8_t scale);

  /**
   * @brief Checks for new magnetometer data.
   *
   * @return True if a new value is available. False otherwise.
   */
  bool magnetHasNewData();

  /**
   * @brief Sets magnetometer to get measures with a sample rate.
   * 
   * When in continuous mode, magnetometer emasures are automatically taken
   * and saved. To get the last measure, use readMagnetX(), readMagnetY() and
   * readMagnetZ().
   * 
   * To define a sample rate, use setMagnetometerDataRate().
   * 
   * @see setMagnetometerDataRate()
   * @see readMagnetX()
   * @see readMagnetY()
   * @see readMagnetZ()
   */
  void setMagnetometerContinuousMode();

  /**
   * @brief Sets magnetometer to operate in single shot mode.
   * 
   * When in single shot mode, magnetometer will just take a measure when
   * requested. To request a measure use readMagnetX(), readMagnetY() and
   * readMagnetZ().
   * 
   * @see readMagnetX()
   * @see readMagnetY()
   * @see readMagnetZ()
   */
  void setMagnetometerSingleShot();

  /**
   * @brief Sets the hard iron offset correction.
   * 
   * Magnetized ferromagnetics near the magnetometer causes a bias in measures.
   * Use a proper software to calculate the hard iron correction and set the
   * calibration factors.
   * 
   * This library has an example, <a href="https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/mcalib/mcalib.ino"> mcalib </a>,
   * that operates in pair with a chrome application to calibrate hard iron and
   * soft iron interferences.
   * 
   * @param x X axis hard iron offset.
   * @param y Y axis hard iron offset.
   * @param z Z axis hard iron offset.
   * 
   * @see setMagnetScale()
   */
  void setMagnetOffset(float x, float y, float z);

  /**
   * @brief Sets the soft iron scale correction.
   * 
   * Unmagnetized ferromagnetics near the magnetometer causes a deformation in
   * measures. Use a proper software to calculate the soft iron correction and
   * set the calibration factors.
   * 
   * This library has an example, <a href="https://github.com/circuitar/Nanoshield_IMU/blob/master/examples/mcalib/mcalib.ino"> mcalib </a>,
   * that operates in pair with a chrome application to calibrate hard iron and
   * soft iron interferences.
   * 
   * @param x X axis soft iron scale.
   * @param y Y axis soft iron scale.
   * @param z Z axis soft iron scale.
   */
  void setMagnetScale(float x, float y, float z);

  /**
   * @brief Gets the angle between earth magnetic north and magnetometer X axis.
   * 
   * @return The angle in radians between earth magnetic north and magnetometer
   * X axis.
   */
  float heading();

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
   * @brief Gets the raw magnetic field measured on X axis.
   * 
   * To use a measure in gauss unit, use readMagnetX().
   * 
   * @return 16 bit integer measured from CI.
   * 
   * @see readMagnetX()
   */
  int16_t magnetRawX();

  /**
   * @brief Gets the raw magnetic field measured on Y axis.
   * 
   * To use a measure in gauss unit, use readMagnetY().
   * 
   * @return 16 bit integer measured from CI.
   * 
   * @see readMagnetY()
   */
  int16_t magnetRawY();

  /**
   * @brief Gets the raw magnetic field measured on Z axis.
   * 
   * To use a measure in gauss unit, use readMagnetZ().
   * 
   * @return 16 bit integer measured from CI.
   * 
   * @see readMagnetZ()
   */
  int16_t magnetRawZ();

  /**
   * @brief Sets the source for interrupt 1 pin.
   * 
   * Possible values:
   * - LSM303D_INT1_BOOT: Notify boot complete.
   * - LSM303D_INT1_CLICK: Click generator interrupt.
   * - LSM303D_INT1_IG1: Inertial interrupt generator 1.
   * - LSM303D_INT1_IG2: Inertial interrupt generator 2.
   * - LSM303D_INT1_IGM: Magnetic interrut generator.
   * - LSM303D_INT1_DRDY_A: Accelerometer data ready signal.
   *     Note that data ready interruption works if data rate is 3.125Hz or 
   *     6.25Hz.
   * - LSM303D_INT1_DRDY_M: Magnetometer data ready signal.
   *     Note that data ready interruption works if data rate is 3.125Hz or 
   *     6.25Hz.
   * - LSM303D_INT1_EMPTY: FIFO empty indication.
   * 
   * To attach a interruption to any arduino pin, use the PCInt feature of
   * atmega 328. PinChangeInterrupt <https://github.com/NicoHood/PinChangeInterrupt>
   * is a library that implements this feature to various microcontrollers.
   * 
   * @param src The source for interruption 1.
   * 
   * @see setAccelerometerDataRate()
   * @see setMagnetometerDataRate()
   */
  void setInterrupt1Source(int8_t src);

  /**
   * @brief Resets the INT1 register.
   * 
   * Avoid that the interrupt signal stops after some triggers.
   */
  void resetInterrupt1();

  /**
   * @brief Sets the source for interrupt 2 pin.
   * 
   * Possible values:
   * - LSM303D_INT2_CLICK:
   * - LSM303D_INT2_IG1:
   * - LSM303D_INT2_IG2:
   * - LSM303D_INT2_IGM:
   * - LSM303D_INT2_DRDY_A:
   *     Note that data ready interruption works if data rate is 3.125Hz or 
   *     6.25Hz.
   * - LSM303D_INT2_DRDY_M:
   *     Note that data ready interruption works if data rate is 3.125Hz or 
   *     6.25Hz.
   * - LSM303D_INT2_OVERRUN:
   * - LSM303D_INT2_FTH:
   * 
   * To attach a interruption to any arduino pin, use the PCInt feature of
   * atmega 328. PinChangeInterrupt <https://github.com/NicoHood/PinChangeInterrupt>
   * is a library that implements this feature to various microcontrollers.
   * 
   * @param src The source for interruption 2.
   * 
   * @see setAccelerometerDataRate()
   * @see setMagnetometerDataRate()
   */
  void setInterrupt2Source(int8_t src);

  /**
   * @brief Resets the INT2 register.
   * 
   * Avoid that the interrupt signal stops after some triggers.
   */
  void resetInterrupt2();

  /**
   * @brief Sets the source for gyroscope interrupt.
   * 
   * TODO COMPLETE DOCUMENTATION.
   * 
   * @param src The source for gyroscope interruption.
   */
  void setGyroInterruptSource(int8_t src);

  /**
   * @brief Resets the gyroscope INT register.
   * 
   * Avoid that the interrupt signal stops after some triggers.
   */
  void resetGyroInterrupt();

  /**
   * @brief Enables the accelerometer buffer.
   * 
   * The accelerometer buffer has five operation modes:
   * - Bypass mode: in bypass mode, the fifo is not operational and for this
   *     reason it remains empty.
   * - FIFO mode: In FIFO mode, data from X, Y and Z channels are stored in the
   *     FIFO. A FIFO threshold/overrun interrupt can be enabled in order to be
   *     raised when the FIFO is filled to the level specified by the internal
   *     register. The FIFO continues filling until it is full. When full, the
   *     FIFO stops collecting data from the input channels. After all data is
   *     read from FIFO, call resetAccelBuffer() to start reading again.
   * - Stream mode: In Stream mode, data from X, Y and Z measurements are
   *     stored in the FIFO. A FIFO threshold interrupt can be enabled and set
   *     as in FIFO mode.The FIFO continues filling until it’s full. When full,
   *     the FIFO discards the older data as the new arrive.
   * - Stream-to-FIFO mode: In Stream-to-FIFO mode, data from X, Y and Z
   *     measurements are stored in the FIFO. A FIFO threshold interrupt can be
   *     enabled in order to be raised when the FIFO is filled to the level
   *     specified by the internal register. The FIFO continues filling until
   *     it’s full. When full, the FIFO discards the older data as the new
   *     arrive. Once a trigger event occurs, the FIFO starts operating in FIFO
   *     mode.
   * - Bypass-to-Stream mode: In Bypass-to-Stream mode, the FIFO starts 
   *     operating in Bypass mode and once a trigger event occurs (related to
   *     LSM303D_IG_CFG1 register events), the FIFO starts operating in Stream
   *     mode.
   *     
   * A threshold can be set in order to reduce the FIFO size. The maximum size
   * is 32 but, as threshold is pre-indexed, it is represented as 31. In the
   * same way, a FIFO with only 2 positions has a threshold 1, and a 3 positions
   * FIFO has a threshold 2.
   * 
   * @param mode Optional. The default operation mode is FIFO mode.
   * @param threshold Optional. The default threshold is 31, which uses all the
   *                  FIFO size.
   *                  
   * @see disableAccelBuffer()
   * @see resetAccelBuffer()
   * @see getBufferCount()
   */
  void enableAccelBuffer(int8_t mode = LSM303D_FIFO, int8_t threshold = 31);

  /**
   * @brief Disable the accelerometer buffer.
   * 
   * When the accelerometer buffer is disable, the last buffer mode is kept.
   * In case of reenabling buffer, it will operate in last buffer mode and
   * threshold.
   */
  void disableAccelBuffer();

  /**
   * @brief Gets how many elements are in accelerometer buffer.
   * 
   * @return The element count in accelerometer buffer.
   */
  int getBufferCount();

  /**
   * @brief Cleans all FIFO data and restarts it.
   *  
   * The reset proccess is basically put FIFO in bypass mode and change it back
   * to the mode selected by the user. It causes to clean all the data in FIFO
   * and restarts the operation.
   * 
   * It is necessary in FIFO mode, where the buffer is blocked once the thereshold
   * is reached, or a overrun occur. To unblock the buffer a reset must be done.
   */
  void resetAccelBuffer();

  /**
   * @brief Sets the interrupt generator 1 operation mode
   * 
   * Possible operation modes:
   * - LSM303D_INTMODE_OR: trigger interruption on OR combination of specified 
   *     zones.
   * - LSM303D_INTMODE_MOVEMENT: pulses interruption signal if acceleration 
   *     vector moves from an unknown zone to a known zone.
   * - LSM303D_INTMODE_AND: trigger interruption on AND conbination of 
   *     specified zones.
   * - LSM303D_INTMODE_POSITION: keeps interruption signal while acceleration
   *     vector stays inside a known zone.
   * 
   * @param mode Operation mode of the interrupt generator.
   */
  void setAccelIntGenerator1Mode(int8_t mode);

  /**
   * @brief Adds an axis to the interrupt generator 1 specified zones.
   * 
   * Possible values are:
   * - LSM303D_ZONE_ZH: Positive part of Z axis.
   * - LSM303D_ZONE_ZL: Negative part of Z axis.
   * - LSM303D_ZONE_YH: Positive part of Y axis.
   * - LSM303D_ZONE_YL: Negative part of Y axis.
   * - LSM303D_ZONE_XH: Positive part of X axis.
   * - LSM303D_ZONE_XL: Negative part of X axis.
   * 
   * @param zone Axis to add to zones.
   */
  void addToAccelIntGenerator1Zone(int8_t zone);

  /**
   * @brief Removes an axis from the interrupt generator 1 specified zones.
   * 
   * Possible values are:
   * - LSM303D_ZONE_ZH: Positive part of Z axis.
   * - LSM303D_ZONE_ZL: Negative part of Z axis.
   * - LSM303D_ZONE_YH: Positive part of Y axis.
   * - LSM303D_ZONE_YL: Negative part of Y axis.
   * - LSM303D_ZONE_XH: Positive part of X axis.
   * - LSM303D_ZONE_XL: Negative part of X axis.
   * 
   * @param zone Axis to remove from zones.
   */
  void removeFromAccelIntGenerator1Zone(int8_t zone);

  /**
   * @brief Sets the threshold on all interrupt generator 1 specified zones.
   * 
   * The interruption generator will only recognize accelerations greater than
   * the threshold in all specified zones.
   * 
   * @param threshold The acceleration threshold to trigger interruption.
   */
  void setAccelIntGenerator1Threshold(float threshold);

  /**
   * @brief Sets the duration of the interruption pulse to interrupt generator 1.
   * 
   * @param duration Interruption pulse duration in periods of data rate.
   */
  void setAccelIntGenerator1Duration(int8_t duration);

  /**
   * @brief Gets the source of the interrupt generator 1 request.
   * 
   * The source information can be extracted by the value doing an and operation
   * between the value and a mask. If the result is greater than 0, then the
   * flag is set. Example:
   * 
   * // Checking if detected acceleration vector has components in positive Z axis.
   * Nanoshield imu;
   * 
   * [...]
   * 
   * int igsource = imu.getAccelIntGenerator1Source();
   * if((igsource & LSM303D_ZONE_ZH) > 0) {
   *   Serial.println("Movement on positive Z axis!");
   * }
   * 
   * The available flags to check are:
   * - LSM303D_INT_ACTIVE: An interruption or more has occurred. This bits
   *     clears automatically after it is read.
   * - LSM303D_ZONE_ZH: Acceleration on positive Z axis detected.
   * - LSM303D_ZONE_ZL: Acceleration on negative Z axis detected.
   * - LSM303D_ZONE_YH: Acceleration on positive Y axis detected.
   * - LSM303D_ZONE_YL: Acceleration on negative Y axis detected.
   * - LSM303D_ZONE_XH: Acceleration on positive X axis detected.
   * - LSM303D_ZONE_XL: Acceleration on negative X axis detected.
   * 
   * @return The value on interrut generator 1 source register.
   */
  int8_t getAccelIntGenerator1Source();

  /**
   * @brief Sets the interrupt generator 2 operation mode
   * 
   * Possible operation modes:
   * - LSM303D_INTMODE_OR: trigger interruption on OR combination of specified 
   *     zones.
   * - LSM303D_INTMODE_MOVEMENT: pulses interruption signal if acceleration 
   *     vector moves from an unknown zone to a known zone.
   * - LSM303D_INTMODE_AND: trigger interruption on AND conbination of 
   *     specified zones.
   * - LSM303D_INTMODE_POSITION: keeps interruption signal while acceleration
   *     vector stays inside a known zone.
   * 
   * @param mode Operation mode of the interrupt generator.
   */
  void setAccelIntGenerator2Mode(int8_t mode);

  /**
   * @brief Adds an axis to the interrupt generator 2 specified zones.
   * 
   * Possible values are:
   * - LSM303D_ZONE_ZH: Positive part of Z axis.
   * - LSM303D_ZONE_ZL: Negative part of Z axis.
   * - LSM303D_ZONE_YH: Positive part of Y axis.
   * - LSM303D_ZONE_YL: Negative part of Y axis.
   * - LSM303D_ZONE_XH: Positive part of X axis.
   * - LSM303D_ZONE_XL: Negative part of X axis.
   * 
   * @param zone [description]
   */
  void addToAccelIntGenerator2Zone(int8_t zone);

  /**
   * @brief Removes an axis from the interrupt generator 2 specified zones.
   * 
   * Possible values are:
   * - LSM303D_ZONE_ZH: Positive part of Z axis.
   * - LSM303D_ZONE_ZL: Negative part of Z axis.
   * - LSM303D_ZONE_YH: Positive part of Y axis.
   * - LSM303D_ZONE_YL: Negative part of Y axis.
   * - LSM303D_ZONE_XH: Positive part of X axis.
   * - LSM303D_ZONE_XL: Negative part of X axis.
   * 
   * @param zone Axis to remove from zones.
   */
  void removeFromAccelIntGenerator2Zone(int8_t zone);

  /**
   * @brief Sets the threshold on all interrupt generator 2 specified zones.
   * 
   * The interruption generator will only recognize accelerations greater than
   * the threshold in all specified zones.
   * 
   * @param threshold The acceleration threshold to trigger interruption.
   */
  void setAccelIntGenerator2Threshold(float threshold);

  /**
   * @brief Sets the duration of the interruption pulse to interrupt generator 2.
   * 
   * @param duration Interruption pulse duration in periods of data rate.
   */
  void setAccelIntGenerator2Duration(int8_t duration);

  /**
   * @brief Gets the source of the interrupt generator 2 request.
   * 
   * The source information can be extracted by the value doing an and operation
   * between the value and a mask. If the result is greater than 0, then the
   * flag is set. Example:
   * 
   * // Checking if detected acceleration vector has components in positive Z axis.
   * Nanoshield imu;
   * 
   * [...]
   * 
   * int igsource = imu.getAccelIntGenerator1Source();
   * if((igsource & LSM303D_ZONE_ZH) > 0) {
   *   Serial.println("Movement on positive Z axis!");
   * }
   * 
   * The available flags to check are:
   * - LSM303D_INT_GENERATED: An interruption or more has occurred. This bits
   *     clears automatically after it is read.
   * - LSM303D_ZONE_ZH: Acceleration on positive Z axis detected.
   * - LSM303D_ZONE_ZL: Acceleration on negative Z axis detected.
   * - LSM303D_ZONE_YH: Acceleration on positive Y axis detected.
   * - LSM303D_ZONE_YL: Acceleration on negative Y axis detected.
   * - LSM303D_ZONE_XH: Acceleration on positive X axis detected.
   * - LSM303D_ZONE_XL: Acceleration on negative X axis detected.
   * 
   * @return The value on interrut generator 2 source register.
   */
  int8_t getAccelIntGenerator2Source();

  /**
   * @brief Sets the gyroscope scale range.
   * 
   * Possible values:
   * - L3GD20H_FS_245: +/- 245dps (8.75 mdps/digit of sensity)
   * - L3GD20H_FS_500: +/- 500dps (17.5 mdps/digit of sensity)
   * - L3GD20H_FS_2000: +/- 2000dps (70 mdps/digit of sensity)
   * 
   * @param scale The scale to adjust the measure.
   */
  void setGyroscopeFullScale(int8_t scale);

  /**
   * @brief Sets the gyroscope data rate and anti-alias filter.
   * 
   * Possible values:
   * - L3GD20H_DRBW_12_5: 12.5Hz data rate and no anti-alias filter.
   * - L3GD20H_DRBW_25: 25Hz data rate and no anti-alias filter.
   * - L3GD20H_DRBW_50_16_6: 50Hz data rate and 16.6Hz anti-alias filter.
   * - L3GD20H_DRBW_100_12_5: 100Hz data rate and 12.5Hz anti-alias filter.
   * - L3GD20H_DRBW_100_25: 100Hz data rate and 25Hz anti-alias filter.
   * - L3GD20H_DRBW_200_12_5: 200Hz data rate and 12.5Hz anti-alias filter.
   * - L3GD20H_DRBW_200_70: 200Hz data rate and 70Hz anti-alias filter.
   * - L3GD20H_DRBW_400_20: 400Hz data rate and 20Hz anti-alias filter.
   * - L3GD20H_DRBW_400_25: 400Hz data rate and 25Hz anti-alias filter.
   * - L3GD20H_DRBW_400_50: 400Hz data rate and 50Hz anti-alias filter.
   * - L3GD20H_DRBW_400_110: 400Hz data rate and 110Hz anti-alias filter.
   * - L3GD20H_DRBW_800_30: 800Hz data rate and 30Hz anti-alias filter.
   * - L3GD20H_DRBW_800_35: 800Hz data rate and 35Hz anti-alias filter.
   * - L3GD20H_DRBW_800_100: 800Hz data rate and 100Hz anti-alias filter.
   * 
   * @param drate Sampling frequency with respective anti-alias filter.
   */
  void setGyroscopeDataRate(int16_t drate);

  // Not working!
  void enableGyroBuffer(int8_t mode, int8_t threshold = 31);

  // Not working!
  void resetGyroBuffer();

  // Not working!
  void disableGyroBuffer();

  // Not working!
  bool isGyroBufferEmpty();

  int8_t getGyroBufferCount();

  /**
   * @brief Checks for new gyroscope data.
   *
   * @return True if a new value is available. False otherwise.
   */
  bool gyroHasNewData();

  /**
   * @brief Gets the last angular speed measured around X axis.
   * 
   * @return The angular speed in degrees/second.
   */
  float readGyroX();

  /**
   * @brief Gets the last angular speed measured around Y axis.
   * 
   * @return The angular speed in degrees/second.
   */
  float readGyroY();

  /**
   * @brief Gets the last angular speed measured around Z axis.
   * 
   * @return The angular speed in degrees/second.
   */
  float readGyroZ();

  int16_t gyroRawX();
  int16_t gyroRawY();
  int16_t gyroRawZ();

  /**
   * @brief Writes a byte to any accelerometer register.
   * 
   * @param addr IC I2C address. Use lsm303dAddress or l3gd20hAddress.
   * @param reg Register address.
   * @param value Value to write.
   */
  void writeToRegister(int8_t addr, int8_t reg, int8_t value);

  /**
   * @brief Reads 16bits from any accelerometer register.
   * 
   * @param addr IC I2C address. Use lsm303dAddress or l3gd20hAddress.
   * @param reg Register to read.
   * @return 16bits value read.
   */
  int8_t readFromRegister(int8_t addr, int8_t reg);

  /**
   * @brief Checks the status of the last I2C communication with the Nanoshield.
   * 
   * @return The status returned by the last Wire.endTransmission().
   */
  int i2cStatus();

  int8_t lsm303dAddress;
  int8_t l3gd20hAddress;
protected:

  int8_t regCtrl0;
  int8_t regCtrl1;
  int8_t regCtrl2;
  int8_t regCtrl3;
  int8_t regCtrl4;
  int8_t regCtrl5;
  int8_t regCtrl6;
  int8_t regCtrl7;
  int8_t aFifoCtrl;
  int8_t igCfg1;
  int8_t igThs1;
  int8_t igDur1;
  int8_t igCfg2;
  int8_t igThs2;
  int8_t igDur2;

  int8_t gyroCtrl1;
  int8_t gyroCtrl3;
  int8_t gyroCtrl4;
  int8_t gyroCtrl5;
  int8_t gFifoCtrl;

  int i2cError;
  int8_t accelScale;
  int8_t magnetScale;
  int16_t gyroScale;
  float softIronX;
  float softIronY;
  float softIronZ;
  float hardIronX;
  float hardIronY;
  float hardIronZ;
  bool hasBegun;

  void writeIfHasBegun(int8_t addr, int8_t reg, int8_t value);
};


#endif