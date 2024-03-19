/*
 * ICM42688.h
 *
 *  Created on: Mar 18, 2024
 *      Author: mohan
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_

#include "main.h"
#include <stdint.h>
#include <stddef.h>

class ICM42688
{
  public:

    enum GyroFS : uint8_t {
      dps2000 = 0x00,
      dps1000 = 0x01,
      dps500 = 0x02,
      dps250 = 0x03,
      dps125 = 0x04,
      dps62_5 = 0x05,
      dps31_25 = 0x06,
      dps15_625 = 0x07
    };

    enum AccelFS : uint8_t {
      gpm16 = 0x00,
      gpm8 = 0x01,
      gpm4 = 0x02,
      gpm2 = 0x03
    };

    /**
     * @brief      Constructor for I2C communication
     *
     * @param      bus      I2C bus
     * @param[in]  address  Address of ICM 42688-p device
     */
    ICM42688(uint8_t address);

    /**
     * @brief      Initialize the device.
     *
     * @return     ret < 0 if error
     */
    int init();

    /**
     * @brief      Sets the full scale range for the accelerometer
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
    int setAccelFS(AccelFS fssel);

    /**
     * @brief      Sets the full scale range for the gyro
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
    int setGyroFS(GyroFS fssel);

    /**
     * @brief      Transfers data from ICM 42688-p to microcontroller.
     *             Must be called to access new measurements.
     *
     * @return     ret < 0 if error
     */
    int getAGT();

    /**
     * @brief      Get accelerometer data, per axis
     *
     * @return     Acceleration in g's
     */
    float accX() const { return _acc[0]; }
    float accY() const { return _acc[1]; }
    float accZ() const { return _acc[2]; }

    /**
     * @brief      Get gyro data, per axis
     *
     * @return     Angular velocity in dps
     */
    float gyrX() const { return _gyr[0]; }
    float gyrY() const { return _gyr[1]; }
    float gyrZ() const { return _gyr[2]; }

    /**
     * @brief      Get temperature of gyro die
     *
     * @return     Temperature in Celsius
     */
    float temp() const { return _t; }

    int calibrateGyro();


  protected:

    // I2C Communication
    uint8_t _address = 0;
    I2C_HandleTypeDef hi2c1;
    HAL_StatusTypeDef ret;

    // temp, accel xyz, gyro xyz
    int16_t _rawMeas[7];

    // buffer for reading from sensor
    uint8_t _buffer[15] = {};

    // data buffer
    float _t = 0.0f;
    float _acc[3] = {};
    float _gyr[3] = {};

    // Full scale resolution factors
    float _accelScale = 0.0f;
    float _gyroScale = 0.0f;

    // Full scale selections
    AccelFS _accelFS;
    GyroFS _gyroFS;

    // Accel calibration
    float _accB[3] = {};
    float _accS[3] = {1.0f, 1.0f, 1.0f};

    // Gyro calibration
    float _gyroBD[3] = {};
    float _gyrB[3] = {};

    // Constants
    static constexpr uint8_t WHO_AM_I = 0x47; ///< expected value in UB0_REG_WHO_AM_I reg
    static constexpr int NUM_CALIB_SAMPLES = 1000; ///< for gyro/accel bias calib

    // Conversion formula to get temperature in Celsius (Sec 4.13)
    static constexpr float TEMP_DATA_REG_SCALE = 132.48f;
    static constexpr float TEMP_OFFSET = 25.0f;

    uint8_t _bank = 0; ///< current user bank

    /* 
     */
    void MX_I2C1_Init(void);

    /*
     */
    int writeRegister(uint8_t subAddress, uint8_t data);

    /*
     */
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);

    /*
     */
    int setBank(uint8_t bank);

    /**
     * @brief      Software reset of the device
     */
    void reset();

    /**
     * @brief      Read the WHO_AM_I register
     *
     * @return     Value of WHO_AM_I register
     */
    uint8_t whoAmI();
};

#endif /* INC_ICM42688_H_ */
