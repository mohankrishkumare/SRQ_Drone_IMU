# SRQ_Drone_IMU

## Overview

This firmware is developed for the STM32F411CE microcontroller unit (MCU) using STM32CubeIDE. It facilitates the acquisition of accelerometer, gyroscope and temperature readings from the ICM-42688-P inertial measurement unit (IMU) and prints the data to a serial monitor.

## Project Core Structure

The project directory is structured as follows:

* Core/
    * Inc/
        * registers.h: Contains register addresses for the ICM-42688-P IMU.
        * ICM42688.h: Header file with class declaration for interfacing with the IMU.
        * main.h: Header file for main.cpp.
        * ...
    * Src/
        * ICM42688.cpp: Implementation of functions to interface with the ICM-42688-P IMU.
        * main.cpp: Entry point of the firmware, responsible for initialization and data transmission.
        * ...
* Debug/: Debug build output directory.
* Drivers/: Contains CMSIS and STM32CubeF4 HAL driver files.

## Usage

To use this firmware:

* Clone the repository.
* Open the project in STM32CubeIDE.
* Build the project.
* Flash the firmware onto the STM32F411CE MCU using ST-LINK.
* Connect a serial terminal (Eg: PuTTY) to VCP of ST-LINK to view IMU data.

## Dependencies

* STM32CubeF4 HAL drivers
* I2C and UART drivers initialized using STM32CubeMX

## MCU Architecture

The STM32F411CE MCU features an ARM Cortex-M4 core with DSP and FPU capabilities, making it well-suited for digital signal processing applications like drone control.

## License

This project is licensed under the MIT License.
