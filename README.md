# IoT Water Control System Using ESP32

This repository contains the code for a capstone project – an IoT Water Control System using an ESP32 board. The project monitors water levels using ultrasonic sensors and measures water flow using YF-S201 flow sensors. It then controls a motor and three water pumps based on sensor data and commands received from a Firebase Realtime Database. Data is exchanged with Firebase in real time for monitoring and control.

## Table of Contents

- [Overview](#overview)
- [Hardware Setup](#hardware-setup)
  - [ESP32 Board](#esp32-board)
  - [Ultrasonic Sensors](#ultrasonic-sensors)
  - [Water Flow Sensors (YF-S201)](#water-flow-sensors-yf-s201)
  - [Motor and Water Pumps](#motor-and-water-pumps)
- [Software Setup](#software-setup)
  - [PlatformIO Configuration](#platformio-configuration)
  - [Libraries Used](#libraries-used)
- [Firebase Integration](#firebase-integration)
- [Code Structure](#code-structure)
- [Usage](#usage)
- [Important Notes](#important-notes)
- [Project Context](#project-context)

## Overview

- **Ultrasonic Sensors:** Three sensors measure distance and are connected through different serial interfaces.
- **Water Flow Sensors:** Three YF-S201 sensors measure water flow via pulse generation. The pulse count is used to calculate the flow rate in L/min.
- **Motor & Water Pumps:** A motor is PWM controlled and three water pumps are switched ON/OFF based on commands from Firebase.
- **Firebase:** Transfers sensor data and control commands in real time.

## Hardware Setup

### ESP32 Board

- **Board:** NodeMCU ESP32 (38-pin)
- **Voltage Levels:** Although power is supplied via a 5V output (VIN), all GPIO signals must be at 3.3V.  
  **Important:** 5V sensor outputs must be level shifted (using a level shifter or a voltage divider) to 3.3V to protect the ESP32.

### Ultrasonic Sensors

- **Sensor 1:**  
  - Uses HardwareSerial2 (example: RX = GPIO16, TX = GPIO17)

- **Sensor 2:**  
  - Uses HardwareSerial1 (example: RX = GPIO4, TX = GPIO15)

- **Sensor 3:**  
  - Uses SoftwareSerial provided by the EspSoftwareSerial library (example: RX = GPIO18, TX = GPIO19)

### Water Flow Sensors (YF-S201)

- **Quantity:** 3 sensors  
- **Connections:**  
  - **Power:** Connect VCC and GND from a 5V source.  
  - **Output:**  
    - Flow Sensor 1 → GPIO34  
    - Flow Sensor 2 → GPIO35  
    - Flow Sensor 3 → GPIO39  
- **Level Shifting:**  
  Use a level shifter or voltage divider for each sensor’s output to convert the 5V pulses into 3.3V signals that are safe for the ESP32.

### Motor and Water Pumps

- **Motor Control:**  
  Controlled using PWM on defined pins (e.g., motor control on GPIO27, GPIO26, with PWM enabled on GPIO14).  
- **Water Pumps:**  
  - Pump 1 → GPIO32  
  - Pump 2 → GPIO33  
  - Pump 3 → GPIO25  
  Controlled by digital output commands (ON/OFF) according to instructions received from Firebase.

## Software Setup

### PlatformIO Configuration

The project uses PlatformIO. Below is an example `platformio.ini` file:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
upload_speed = 115200
lib_deps = 
    mobizt/Firebase Arduino Client Library for ESP8266 and ESP32@^4.4.17
    mobizt/FirebaseClient@^2.0.3
    plerup/EspSoftwareSerial@^8.2.0
```

### Libraries Used

- **Arduino**
- **WiFi**
- **Firebase_ESP_Client**
- **FirebaseClient**
- **HardwareSerial**
- **SoftwareSerial** (via EspSoftwareSerial library)

## Firebase Integration

- **Configuration:**  
  Firebase is initialized with the API key, database URL, email, and password in the code.
- **Data Paths:**  
  - Ultrasonic sensor readings are sent to `/TEST/s1`, `/TEST/s2`, and `/TEST/s3`.
  - Water flow sensor data is sent to `/TEST/flow1`, `/TEST/flow2`, and `/TEST/flow3`.
  - Commands from Firebase control the motor and water pump operations.

## Code Structure

The main code in `main.cpp` consists of:

- **WiFi & Firebase Initialization:**  
  The code connects to WiFi and initializes Firebase.
  
- **Hardware Initialization (`initHardware()`):**  
  Sets up motor, pump, ultrasonic sensors, and water flow sensors (with ISRs for pulse counting).
  
- **Sensor Reading Functions:**  
  - `readUltrasonic(Stream &stream, float &dist)` – Reads sensor data with a 100ms timeout.  
  - Water flow conversion using pulse count (conversion factor: L/min = pulse count / 15).
  
- **Control Loop (`loop()`):**  
  Executes every 2 seconds to read sensor data, send it to Firebase, and retrieve control commands for motor and pumps.
  
- **Actuation Functions:**  
  Functions that control motor operation and water pump states based on Firebase commands.

## Usage

1. **Hardware Assembly:**  
   Assemble the circuit according to the hardware setup above. Ensure level shifting circuits are in place for YF-S201 sensors.
   
2. **Software Configuration:**  
   Update Firebase credentials and adjust pin mappings as needed in `main.cpp`. Use the provided `platformio.ini` for building the project.
   
3. **Uploading & Debugging:**  
   Upload the code to your ESP32 board using PlatformIO. Open the Serial Monitor at 115200 baud to view debugging and sensor data. Monitor your Firebase Realtime Database for incoming data and control commands.

## Important Notes

- **Voltage Compatibility:**  
  Sensor outputs must be shifted from 5V to 3.3V using a level shifter or voltage divider to prevent damage to the ESP32 GPIO.
  
- **Sensor Timeouts and Conversions:**  
  Timeout settings and flow conversion factors are tuned as per the sensor specifications but may need adjustments based on your setup.
  
- **Firebase:**  
  Ensure your Firebase rules and credentials are correctly set up for both reading and writing data.

## Project Context

This project is developed as a capstone project for my final assignment. The goal is to design and implement an intelligent water control system that:
- Monitors water levels and flow rates.
- Controls water pump operations and motor functions in real-time.
- Provides remote monitoring and control via Firebase.

This detailed documentation outlines the hardware and software setup, ensuring that anyone continuing or reviewing the project can quickly understand the system architecture and functionality.
