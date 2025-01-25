# ESP32 Tank Level & PEM Protection System

## Overview
This ESP32-based project monitors the fill level of a tank and controls a gauge to display the level. Additionally, it ensures the safe operation of a PEM (Proton Exchange Membrane) system by cutting off power if the level is too low or if the temperature exceeds a predefined threshold.

## Features
- **Tank Level Monitoring**: Reads an analog input (GPIO 34) from a sensor measuring the tank's fill level.
- **Gauge Control**: Uses PWM (GPIO 25) to smoothly move the gauge needle between empty (140) and full (225) based on the tank level.
- **Backlight Control**: Blinks a backlight (GPIO 26) when the tank level is low.
- **Power Cutoff (KILL_PIN)**: Disables the PEM system (GPIO 27) if:
  - The tank level is below a critical threshold.
  - The PEM temperature exceeds 60°C.
- **Temperature Monitoring**: Uses a 10kΩ NTC thermistor (GPIO 35) to measure temperature.
- **Noise Reduction**: Implements ADC smoothing using a moving average filter to stabilize readings.

## Hardware Requirements
- **ESP32-WROOM-32D** microcontroller
- **Tank level sensor** (0-3.3V output) connected to GPIO 34
- **IRLB8721 MOSFET** for gauge control (PWM on GPIO 25)
- **Backlight LED** (controlled via GPIO 26)
- **Relay or MOSFET** for power cutoff (controlled via GPIO 27)
- **10kΩ NTC thermistor** with a **10kΩ pull-up resistor** (connected to GPIO 35)

## Pin Configuration
| Function            | GPIO  | Description |
|--------------------|------|-------------|
| Tank Level Sensor | 34   | Reads fill level (0-3.3V) |
| Gauge Control (PWM) | 25   | Controls gauge needle via MOSFET |
| Backlight Control  | 26   | Blinks when level is low |
| Power Cutoff (KILL_PIN) | 27   | Turns PEM system OFF if level is low or temp is high |
| Thermistor Input   | 35   | Reads temperature |

## Installation
1. Clone this repository:
2. Open the code in **Arduino IDE** or **PlatformIO**.
3. Install **ESP32 Board Support** via Board Manager.
4. Connect your ESP32 to your PC and select the correct port.
5. Compile and upload the firmware.

## Usage
- **Monitoring**: The gauge will reflect the tank level based on the sensor input.
- **Low-Level Alert**: The backlight blinks when the level is low.
- **Power Cutoff**: The system disables the PEM power if the level is critically low or the temperature exceeds 60°C.
- **Debugging**: Serial output provides real-time updates on voltage, PWM, temperature, and power status.

## License
This project is open-source and available under the **MIT License**.



