# Project Overview
This project is a collaboration between **KVISpace** and **CUHAR** for the 2025 IREC (Intercollegiate Rocket Engineering Competition). Our team, KVISpace, focuses on developing the **payload** with two main objectives:
  - Demonstrating how a gimbal (gyroscope) stabilizes objects.
  - Predicting the rocket’s landing site.

My primary responsibilities include *electronics development* and *data analysis*. 

## Hardware usage
Microcontroller:
- 1 x Arduino UNO R4 
- 1 x Heltec LoRa ESP32 V2

Module:
- 2 × GY-521 MPU6050: Measures 3-axis acceleration, yaw, and pitch (I2C).
- 1 × GY-273 HMC5883L: Corrects 3-axis acceleration and measures changes in direction relative to the landing site (I2C).
- 1 × BME280: Measures temperature, pressure, humidity, and altitude (I2C).
- 1 × TCA9548A I2C Multiplexer: Expands and organizes I2C communication channels (I2C).
- 1 × NEO-6M GPS: Provides payload location and tracks the flight trajectory (UART).
- 1 × L298N Motor Driver: Controls the gimbal motors (PID control).
- 1 × SD Card Module: Backs up sensor data (SPI).

> [!NOTE]
> The Arduino sketch exceeds 32 KB, so the Arduino UNO R3’s memory is insufficient.
> The multiplexer is not strictly necessary but was included to improve sensor fault tolerance and allow for future sensor addition.

## Contents in this project
- PCB and Sensors footprints
  Designed using KiCad 9.0. Files are provided in .kicad_sch (schematic), .kicad_pcb (PCB layout), and .kicad_sym (symbols/footprints) formats.
- Arduino code. Special thanks to [@Abhi-Ya](https://github.com/Abhi-Ya)
  The code is written in .ino files for Arduino UNO R4 and Heltec LoRa ESP32 V2.
- AI landing site prediction
  I developed a model using two fully connected LSTM layers to predict the landing site based on sequential rocket sensor data. The goal is to enable real-time predictions.
