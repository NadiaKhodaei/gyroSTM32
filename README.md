
# Project Title: STM32 Gyro LED Control

## Introduction
This project aims to control the onboard LEDs of an STM32 board based on the roll, pitch and Yaw angles  obtained from gyroscopic data. The system consists of two main components: a Python script running on a Jetson Nano to process gyro data and send LED control commands, and firmware running on the STM32 board to receive commands and control the LEDs accordingly.
This Python script enables real-time control of LEDs based on gyroscope readings obtained from an STM32 microcontroller board. The script communicates with the STM32 board via a serial connection, reads gyroscope data, calibrates it, and adjusts LED behavior accordingly. Additionally, it implements features for gyro drift monitoring and online noise measurement.

## Functionality 
- LED Control: LEDs change color and intensity based on gyroscope readings (roll and pitch angles) received from the STM32 board.
- Gyro Calibration: The script calibrates gyroscope data by computing baseline values for roll, pitch, and yaw angles.
- Gyro Drift Monitoring: Continuous monitoring of gyro drift ensures accuracy of gyroscope readings over time.
- Online Noise Measurement: The script measures noise levels in gyroscope data and provides real-time visualization for analysis.

## Dependencies
-Python 3.x
-NumPy
-Matplotlib
-PySerial

### Block Diagram:
Apologies for the confusion. Here's a block diagram illustrating the software components of the project using VS Code:

```
          +---------------------------------------------------------+
          |                      Jetson Nano                        |
          |                    Python Environment                  |
          +---------------------------------------------------------+
                                   |
                      +-------------------------------------+
                      |            uncalibrated.py         |
                      +-------------------------------------+
                                   |
                      +-------------------------------------+
                      |             PySerial Library        |
                      +-------------------------------------+
                                   |
                      +-------------------------------------+
                      |         USB Serial Interface        |
                      |              (UART)                 |
                      +-------------------------------------+
                                   |
                      +-------------------------------------+
                      |          STM32 Board (Firmware)     |
                      |          C/C++ Development          |
                      +-------------------------------------+
                                   |
                      +-------------------------------------+
                      |         USB Serial Interface        |
                      |              (UART)                 |
                      +-------------------------------------+
```

In this diagram:

- The Jetson Nano runs a Python environment where the `uncalibrated.py` or `calibration.py`  is executed.
- The Python script utilizes the PySerial library to establish communication with the STM32 board over a USB serial interface (UART).
- The STM32 board runs firmware developed in C/C++.
- The firmware interfaces with the USB serial interface (UART) to communicate with the Python script.

This diagram illustrates the flow of data and commands between the software components of the project.

## Instructions
To run the code:
Connect the gyroscope sensor and LEDs to the STM32 microcontroller board.

Install Dependencies: Ensure Python 3.x is installed on your system. Install required Python libraries using pip:

pip install numpy matplotlib pyserial

Serial Port Configuration: Modify the port_name, baud_rate, and timeout variables in the script to match your serial port settings.

Upload Firmware: Upload the provided firmware to the STM32 board using the appropriate Integrated Development Environment (IDE) such as STM32CubeIDE:

   - Open STM32CubeIDE and create a new project for your microcontroller.
   - Import or create the necessary source files for your project, including the provided firmware.
   - Configure the project settings, such as target device, toolchain, and build configurations.
   - Build the project to generate the firmware binary.
   - Connect the STM32 board to your computer using a USB cable.
   - Flash the firmware binary to the STM32 board using the IDE's programming tool (e.g., ST-LINK) and follow the instructions provided by the IDE.

Run Script: Execute the Python script on your computer.
## Usage :
- Run the Python script on your Linux based computer Or jetson, please note that if your testing the code on jetson with remote access comment or remove real-time plot  section .
- Observe the behavior of LEDs connected to the STM32 board, which change based on gyroscope readings.
- Monitor gyro drift and noise levels in the console output for system health assessment.
- Ensure proper corrective action if gyro drift or noise levels exceed acceptable thresholds.

## Troubleshooting :
- If the script fails to establish a serial connection, check the serial port settings and ensure the correct port is specified.
- If gyro drift or noise levels are abnormal, consider recalibrating the gyroscope or investigating potential hardware issues.



## Table of Contents
1. `uncalibrated.py`: Python script running on the Jetson Nano to process gyro data and control LEDs.
2. `stm32_firmware.ino`: Firmware for the STM32 board to receive LED control commands and set LEDs accordingly.

## Credits
- PySerial library: [https://github.com/pyserial/pyserial](https://github.com/pyserial/pyserial)
