# Bomba-Pro: Autonomous Micro Sumo Robot

A high-performance micro sumo robot powered by the ESP32-S3-MINI-1U microcontroller with advanced autonomous navigation, real-time sensor fusion, and intelligent obstacle avoidance.

## 🎯 Project Overview

Bomba-Pro is a compact sumo robot designed for competitive micro-sumo competitions. It features multiple autonomous operating modes with adaptive PID control, gyroscope-based navigation, and real-time distance sensing across 7 directions.

## ✨ Key Features

### Operating Modes
- **Mode 1: Opponent Tracking** - Pursues opponents with adaptive speed control and obstacle awareness
- **Mode 2: Distance Control** - Maintains precise distance from targets before engaging
- **Mode 3: Gyro Navigation** - Rotates to specified angle then advances forward
- **Mode 4: Gyro Demo** - Pure rotation testing mode

### Hardware Capabilities
- **7× VL53L1X Time-of-Flight Sensors** - 200° field of view for opponent detection
- **MPU6050 6-Axis IMU** - Precise gyroscope for angle tracking and rotation control
- **7× WS2812C RGB LEDs** - Real-time visual feedback (sensor distances, modes, status)
- **Dual Motor Control** - Independent left/right speed control with PWM
- **IR Receiver** - RC5 protocol for wireless start/stop commands

### Software Features
- **Adaptive PID Control** - Separate tuning profiles for normal and slow-approach modes
- **Real-time Parameter Tuning** - Modify all control constants via web interface over WiFi
- **EEPROM Persistence** - All settings saved across power cycles
- **Non-blocking IR Task** - Dual-core ESP32 handles IR commands without blocking main loop
- **Gyro Calibration** - Automatic bias measurement with visual LED feedback
- **Derivative Filtering** - Alpha-blended low-pass filtering for smooth control response

## 🛠️ Hardware Components

| Component | Part | Notes |
|-----------|------|-------|
| Microcontroller | ESP32-S3-MINI-1U | 240MHz dual-core |
| Distance Sensors | VL53L1X (×7) | ToF distance measurement |
| IMU | MPU6050 | 6-axis gyro + accelerometer |
| Motors | N20 1000 rpm | Fast and strong |
| Motor Driver | TB6612FNG | Enough for N20 motors|
| LED Driver | MCP23008 (GPIO Expander) | Addressable LED control |
| Status LEDs | WS2812C-2020 (×7) | RGB NeoPixel LEDs |
| IR Receiver | Generic 38kHz | RC5 protocol compatible |

## 📊 Control System Architecture

### PID Controller
The robot uses dual PID controllers:
1. **Drive PID** - Controls left/right wheel speed corrections for line tracking
2. **Gyro PID** - Controls rotation to reach target angles

```
Error Input → [P] → 
              [I] → Sum → Output
              [D] (with derivative filter)
```

**Tunable Parameters:**
- Kp (Proportional gain) - Controls response strength
- Kd (Derivative gain) - Controls response smoothness
- slowKp/slowKd - Reduced gains for cautious approach

### Sensor Fusion
- When `en_gyro = true`: Uses gyroscope for angle tracking
- When `en_gyro = false`: Uses 7 distance sensors for opponent detection
- Automatic switching based on operational mode and state

## 🚀 Operating Modes Detailed

### Mode 1: Opponent Tracking
- Rotates in place looking for opponents
- Upon detection, accelerates toward them
- **Slow-down behavior**: Reduces acceleration when obstacle < 100mm for 500ms
- Edge sensors trigger emergency spin to avoid wall contact
- Auto-ramps speed based on target distance

### Mode 2: Distance Control
- Maintains 60mm distance from front obstacle
- Advances slowly if no obstacle detected
- Transitions to Mode 1 after 500ms of no obstacle contact

### Mode 3: Gyro Navigation
- Rotates to `target_angle` (adjustable via tuner, -180° to +180°)
- Once aligned, advances at 25 speed
- Switches to Mode 1 if opponent detected
- Requires gyro calibration for accurate angle tracking

### Mode 4: Gyro Demo
- Continuous rotation at specified gyro gains
- Used for testing gyroscope response and tuning

## 🎮 Control Interface

### Physical Controls
- **Button Press**: Cycle through modes (visual feedback via LED count)
- **Button Hold (700ms+)**: Calibrate gyro (LED animation during calibration)

### Remote Control (IR)
- **Start Command (0x07 address)**: Begin autonomous operation
- **Stop Command (0x07 address)**: Stop and return to manual mode
- **Programming (0x0B address)**: Learn custom RC codes for start/stop

### Web Tuner
Access via WiFi to modify:
- All PID gains in real-time
- Speed thresholds
- Target angles
- Sensor calibration values
- LED brightness

## 📈 Parameter Guidelines

### Speed Parameters
- `base_speed`: 0-100 (maximum forward speed)
- `spin_speed`: 0-100 (rotation speed when searching)
- `ramp_up_step`: Speed acceleration rate (0.75 default)

### PID Tuning
- **Normal Mode**: Kp=75, Kd=4 (aggressive response)
- **Slow Mode**: Kp=15, Kd=0.5 (careful approach)
- **Gyro Mode**: gyroKp=2.4, gyroKd=0.07 (rotation control)

### Distance Thresholds
- `threshold`: Maximum detectable distance (500mm default)
- Obstacle detection: < 100mm
- Approach caution: < 100mm for 500ms
- Direct opponent: < 50mm

## 🔧 Setup & Installation

### Prerequisites
- PlatformIO CLI or VSCode with PlatformIO extension
- Arduino IDE compatible libraries
- ESP32 boards package

### Libraries Required
```
Adafruit_NeoPixel
Adafruit_MCP23X08
Adafruit_MPU6050
VL53L1X_ULD
RC5
Preferences (built-in)
```

### Building & Upload
```bash
pio run -e esp32-s3-devkitc-1
pio run -t upload -e esp32-s3-devkitc-1 --upload-port COM29
```

### First-Time Setup
1. **Calibrate Gyro**: Hold button for 700ms, wait for LED animation
2. **Test Modes**: Press button to cycle through all 4 modes
3. **IR Programming**: Hold button to teach start/stop codes from remote
4. **Tune Parameters**: Connect to web tuner and adjust PID values

## 📝 Code Structure

### Main Files
- `src/main.cpp` - Core control loop and mode selection
- `src/motors.cpp` - Motor PWM control and speed mapping
- `src/sensors.cpp` - Distance sensor reading and error calculation
- `src/start_module.cpp` - IR receiver task and start/stop logic
- `src/tuner.cpp` - Web-based parameter tuning interface
- `lib/RC5/` - Custom RC5 IR protocol decoder

### Key Functions
- `loop()` - Main control cycle (runs ~100Hz)
- `callibrate_gyro()` - Measures and stores gyro bias
- `read_sensors()` - Processes distance data and calculates tracking error
- `pid()` - Generic PID controller implementation
- `drive()` - Sends speed commands to motors

## 📊 Performance Metrics

- **Loop Frequency**: ~100Hz
- **Sensor Update Rate**: 20ms (VL53L1X)
- **Gyro Polling**: Continuous when enabled
- **IR Response Time**: <50ms
- **Total Code Size**: ~400KB (with WiFi stack)
- **Runtime**: ~4-6 hours (estimated, battery dependent)

## 🐛 Troubleshooting

### Robot doesn't respond to IR
- Check IR receiver connection on pin 4
- Verify RC codes are programmed via button hold
- Test with serial monitor: watch `started` flag

### Gyro readings drift
- Recalibrate using button hold
- Keep robot stationary during calibration
- Check MPU6050 mounting for vibration isolation

### Motors won't start
- Verify STBY pin (15) is properly connected
- Check motor PWM pins and enable pins
- Monitor `started` flag in serial output

### Inaccurate distance readings
- Verify all 7 sensors are properly initialized
- Check sensor alignment (must face forward)
- Recalibrate threshold value for your environment

## 🚗 Typical Match Sequence

1. **Power On** - Robot calibrates, displays mode via LED count
2. **IR Start** - Begin autonomous operation, search for opponent
3. **Opponent Detection** - Rotate toward opponent (Modes 1-2)
4. **Approach** - Close distance with adaptive speed control
5. **Collision** - Push opponent out of ring
6. **IR Stop** - Return to idle state for next match

## 📚 References

- [VL53L1X Datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [ESP32-S3 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- RC5 Protocol: Philips remote control standard

## 📄 License

This project is open source. Feel free to use and modify for educational and competitive purposes.

## 🤝 Contributing

Suggestions for improvements:
- Better gyro calibration routines
- Machine learning opponent prediction
- Sensor fusion algorithms
- Network connectivity for telemetry
- Advanced tuning interfaces

---

**Built with ❤️ for competitive robotics**

Last Updated: April 2026
