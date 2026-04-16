<!-- # Bomba-Pro: Autonomous Micro Sumo Robot

A high-performance micro sumo robot powered by the ESP32-S3-MINI-1U microcontroller with advanced autonomous navigation, real-time sensor fusion, and intelligent obstacle avoidance.

## Project Overview

Bomba-Pro is a compact sumo robot designed for competitive micro-sumo competitions. It features multiple autonomous operating modes with adaptive PID control, gyroscope-based navigation, and real-time distance sensing across 7 directions.

## Key Features

### Operating Modes
- **Mode 1: Opponent Tracking** - Pursues opponents with adaptive speed control
- **Mode 2: Distance Control** - Slowly approaches the target and switches to mode 1
- **Mode 3: Gyro Navigation** - Rotates to specified angle then advances forward
<!-- - **Mode 4: Gyro Demo** - Pure rotation testing mode -->

<!-- ### Hardware Capabilities
- **7× VL53L1X Time-of-Flight Sensors** - 200° field of view for opponent detection
- **MPU6050 6-Axis IMU** - Precise gyroscope for angle tracking and rotation control
- **7× WS2812C RGB LEDs** - Real-time visual feedback (sensor distances, modes, status)
- **Dual Motor Control** - Independent left/right speed control with PWM
- **IR Receiver** - RC5 protocol for wireless start/stop commands

### Software Features
- **Adaptive PID Control** - Separate tuning profiles for normal and slow-approach modes
- **Real-time Parameter Tuning** - Modify all control constants via web interface over WiFi
- **EEPROM Persistence** - All settings saved across power cycle    s
- **Non-blocking IR Task** - Dual-core ESP32 handles IR commands without blocking main loop
- **Gyro Calibration** - Automatic bias measurement with visual LED feedback
- **Derivative Filtering** - Alpha-blended low-pass filtering for smooth control response

## Hardware Components

| Component | Part | Notes |
|-----------|------|-------|
| Microcontroller | ESP32-S3-MINI-1U-N4 | 240MHz dual-core, 4Mb flash|
| Distance Sensors | VL53L1X (×7) | ToF distance measurement |
| IMU | MPU6050 | 6-axis gyro + accelerometer |
| Motors | N20 1000 rpm | Fast and strong |
| Motor Driver | TB6612FNG | Enough for N20 motors|
| LED Driver | MCP23008 (GPIO Expander) | Addressable LED control |
| Status LEDs | WS2812C-2020 (×7) | RGB NeoPixel LEDs |
| IR Receiver | TSOP4838 | 38kHz, RC5 protocol compatible |

## TO DO: ADD MORE LATER! -->