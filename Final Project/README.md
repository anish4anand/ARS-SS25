# LockerNinja & Dojo Security System

## Overview
LockerNinja is an Arduino-based security system that detects intrusion attempts inside a locker using multiple onboard sensors. It wirelessly transmits all sensor data to the Dojo, a receiver unit that displays alerts, breach counts, and allows remote resets using an IR remote or a pushbutton.

This project includes two Arduino sketches:
- **LockerNinja (Transmitter)** — Reads sensors and sends alerts  
- **Dojo (Receiver)** — Displays breach count on an LED matrix, runs a stepper motor, and sends reset commands back

Communication is via **nRF24L01** using acknowledgment payloads.

## Features

### LockerNinja (Transmitter)
- Breach detection using **LDR**
- Motion sensing via **ultrasonic sensor**
- Temperature & humidity using **DHT11**
- Sound detection
- 3-axis acceleration using **ADXL345**
- Timestamping using **DS3231 RTC**
- Sends a compact RF24 data packet (struct)
- Supports remote reset from Dojo

### Dojo (Receiver)
- Receives and parses RF24 data in real-time
- Displays breach count (0–3) on **8×8 LED matrix**
- Alert mode for >3 breaches
- Runs a stepper motor when live data is incoming
- Reset via **IR remote** or **reset button**
- Sends reset acknowledgment payloads

## Hardware List

### LockerNinja
- Arduino Uno/Nano  
- nRF24L01  
- DHT11 temperature/humidity sensor  
- LDR with resistor  
- HC-SR04 ultrasonic sensor  
- ADXL345 accelerometer (I2C)  
- DS3231 RTC  
- Sound sensor (digital)  
- 2× Servo motors  
- Breadboard + jumper wires  

### Dojo
- Arduino Uno/Nano  
- nRF24L01  
- 8×8 LED matrix (shift-register version)  
- ULN2003 + Stepper motor  
- IR receiver  
- Pushbutton  

## Folder Structure
```
/LockerNinja/
    LockerNinja.ino
/Dojo/
    Dojo.ino
README.md
```

## Installation & Setup

### 1. Install Required Libraries
- RF24  
- Servo  
- dht  
- IRremote  
- uRTCLib  
- SPI / Wire  

### 2. Wiring
Wire each module according to the pin definitions in the `.ino` files.  
The nRF24L01 must use:
- CE → 9  
- CSN → 8  
- Stable 3.3V  
- Add 10µF capacitor between VCC–GND (recommended)

### 3. Upload
Upload `LockerNinja.ino` to the transmitter and `Dojo.ino` to the receiver.

## System Behavior

### LockerNinja
1. Reads sensors every 2 seconds  
2. Packages data into a struct  
3. Transmits via RF24  
4. Waits for ACK payload  
   - `"1"` = Reset breach count  
   - `"0"` = Continue  

### Dojo
1. Continuously listens for incoming data  
2. Displays breach count on LED matrix  
3. Runs stepper motor when receiving active data  
4. Sends reset commands when remote or button is used  

## Troubleshooting

| Issue | Fix |
|-------|------|
| No RF communication | Check wiring, add capacitor, ensure CE/CSN correct |
| LED matrix flickering | Ensure correct multiplex timing |
| RTC wrong time | Set time using `uRTCLib` examples |
| IR not responding | Verify IR code and update in handleIR() |
| Stepper jitter | Use dedicated 5V supply |

## Future Extensions
- ESP32 cloud notifications  
- SD card logging  
- Lock mechanism control  
- OLED display on Dojo  

