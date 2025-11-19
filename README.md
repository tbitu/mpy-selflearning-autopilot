# 🚤 ESP32 Self-Learning Outboard Autopilot

> **Adaptive Trolling Autopilot for Small Boats**  
> *Powered by MicroPython, PID Control, and BNO055 Sensor Fusion*

![Status](https://img.shields.io/badge/Status-Active-success)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![License](https://img.shields.io/badge/License-MIT-green)

This project is a **self-learning autopilot** designed specifically for outboard trolling. Unlike traditional autopilots that require complex manual tuning, this system uses **Passive System Identification** to learn your boat's steering characteristics and automatically tunes its PID controller for optimal performance.

---

## ✨ Key Features

-   **🧠 Adaptive PID Tuning**: Automatically learns your boat's responsiveness (`deg/s` per `% power`) and adjusts gains on the fly.
-   **🔄 Continuous Control Loop**: Runs a non-blocking 10Hz control loop for smooth, responsive steering in wind and waves.
-   **🧭 9-Axis Sensor Fusion**: Uses the BNO055 in **NDOF mode** (Quaternions) for tilt-compensated, stable heading data.
-   **🌊 Sea State Estimation**: Monitors roll variance to detect rough water and adapt control behavior (future expansion).
-   **💾 Auto-Save**: Automatically saves learned PID gains and settings to NVS memory.
-   **⚡ Non-Blocking**: Fully asynchronous motor control and sensor reading.

---

## 🛠️ Hardware Setup

| Component | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **BNO055 SDA** | `GPIO 21` | I2C Data |
| **BNO055 SCL** | `GPIO 22` | I2C Clock |
| **Motor PWM** | `GPIO 27` | Speed Control (0-100%) |
| **Motor DIR** | `GPIO 26` | Direction (High=Starboard, Low=Port) |
| **Button** | `GPIO 33` | Engage/Disengage (Pull-up) |
| **LED** | `GPIO 19` | Status Indicator |

### Recommended Hardware
-   **Controller**: ESP32 DevKit V1
-   **Sensor**: Adafruit BNO055 (or generic module)
-   **Motor Driver**: BTS7960 (High Current) or L298N (Low Current)
-   **Pump**: 12V Hydraulic Reversing Pump (e.g., Raymarine Type 1)

---

## 🚀 Installation

1.  **Flash MicroPython**: Install the latest MicroPython firmware on your ESP32.
2.  **Upload Files**: Upload the following files to the root directory:
    -   `main.py`: Core logic
    -   `lib/bno055.py`: Sensor driver
    -   `lib/bno055_base.py`: Sensor base driver
    -   `lib/motor.py`: Motor controller
    -   `lib/pid.py`: PID controller
    -   `lib/autotuner.py`: Adaptive tuning logic
    -   `lib/gps_stub.py`: GPS placeholder
3.  **Wiring**: Connect hardware according to the table above.

---

## 🎮 Usage

### 1. Calibration (First Run)
The BNO055 needs calibration.
-   Power on the system.
-   Rotate the sensor/boat in figure-8 motions until the system status indicates calibration (LED behavior can be added for this).
-   *Note: The system saves calibration data automatically.*

### 2. Engaging Autopilot
1.  Point the boat on your desired heading.
2.  Press the **Button** (GPIO 33).
3.  The **LED** will turn ON.
4.  The system will lock onto the current heading.

### 3. Auto-Tuning (Passive)
-   Just drive! The system starts with "Safe" default gains.
-   As it makes corrections, it measures how fast the boat turns.
-   If the boat is sluggish, it increases aggressiveness.
-   If the boat is twitchy, it decreases aggressiveness.
-   Settings are saved every 60 seconds.

### 4. Disengaging
-   Press the **Button** again.
-   The **LED** turns OFF.
-   Motor stops immediately.

---

## 📂 Project Structure

```
├── main.py              # Entry point & Control Loop
├── lib/
│   ├── autotuner.py     # Adaptive PID Logic
│   ├── bno055.py        # BNO055 Driver
│   ├── bno055_base.py   # BNO055 Base Class
│   ├── motor.py         # Non-blocking Motor Control
│   ├── pid.py           # PID Controller
│   └── gps_stub.py      # Future GPS Integration
└── README.md            # This file
```

---

## 🔮 Future Roadmap

-   [ ] **GPS Integration**: Connect NMEA GPS for Course Over Ground (COG) hold.
-   [ ] **Pattern Steering**: Zig-zag and Circle modes for fishing.
-   [ ] **WiFi UI**: Web-based configuration and monitoring.
