# 2-AXIS_BLDC_CAMERA_GIMBAL
A two axis camera gimbal stabilizer with the use of BLDC motors.

## Introduction
This is a "Video-camera image stabilizer" system. It's purpose is to maintain the camera at a given angle (X/Y axes) at all times. This is done with the use of a PID controller that receives the camera's current position by an Accelerometer/Gyroscope MEMS sensor and then outputs the required power and direction to the motors. A "2-axis/one button" joystick is also implemented in order to change the default resting angle. PID values and motor power curve can be changed by a "setup-menu" when connected to a serial terminal. Unfortunately the system is only stable in very slow speeds, an addition of a PD controller with hall-effect position sensors in the motors might solve this problem.

## Hardware
The system's MCU is an Atmega644, the ACC/GYRO is a MPU-9150 and two L6234 chips are used as motor drivers.
## How to install/use
PCB circuit and Schematic are provided in the links below, the MCU needs to be flashed in a different board. You can use AVR Studio in order to compile and upload the hex files. If the circuit is connected to a serial terminal at startup, you can press the joystick-button to access the "setup-menu", all values are registered in the MCU's EEPROM.

Schematics are given in the links bellow:
https://drive.google.com/open?id=1g8CwjlbzEkt50502nUi9NxbfcElKLwDl
