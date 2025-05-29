# Single_Tank_Fuzzy_Control

## Project Description
Fuzzy-PI controller to control the liquid level of a single tank using STM32F4 hardware.

This project implements a hybrid fuzzy-logic and PI controller on an STM32F411VET6 microcontroller to maintain the liquid level in a single tank at a desired setpoint. The firmware reads the tank level via an ultrasonic sensor, applies fuzzy inference rules for non-linear tuning, and adds a PI component to eliminate steady-state error. The resulting control signal drives a PWM‐controlled valve.

## Features
- Hardware implementation on STM32F411VET6  
- Fuzzy rule base for non-linear error handling  
- PI controller component to remove steady-state offset  
- Sensor interface via ultrasonic distance module (SRF05)  
- PWM output to control a proportional valve  
- Configurable fuzzification and PI parameters in firmware  

## Hardware Components
- **Microcontroller**: STM32F411VET6 (STM32F4 Discovery)  
- **Ultrasonic Sensor**: HC-SR04 or equivalent  
- **Power Supply**: 5 V for sensor, 12 V for valve driver

## Prerequisites
- STM32CubeIDE (version 1.8.0 or later)  
- STM32CubeMX for peripheral configuration (optional)  
- ST-LINK debugger/programmer

## Video
Watch my implementation video here: https://drive.google.com/file/d/188v5voxnT75FDu0BoV7C0SMWsRcZnLFV/view?usp=drive_link

