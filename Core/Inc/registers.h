/*
 * registers.h
 *
 *  Created on: Mar 18, 2024
 *      Author: mohan
 */

#ifndef INC_ICM42688_REGISTERS_H_
#define INC_ICM42688_REGISTERS_H_

#include <stdint.h>

namespace ICM42688reg {
  // Accesible from all user banks
  static constexpr uint8_t REG_BANK_SEL = 0x76;

  // User Bank 0
  static constexpr uint8_t UB0_REG_DEVICE_CONFIG = 0x11;
  // break
  static constexpr uint8_t UB0_REG_TEMP_DATA1 = 0x1D;
  // break
  static constexpr uint8_t UB0_REG_PWR_MGMT0 = 0x4E;
  static constexpr uint8_t UB0_REG_GYRO_CONFIG0 = 0x4F;
  static constexpr uint8_t UB0_REG_ACCEL_CONFIG0 = 0x50;
  // break
  static constexpr uint8_t UB0_REG_WHO_AM_I = 0x75;
} // ns ICM42688reg

#endif /* INC_ICM42688_REGISTERS_H_ */
