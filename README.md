# ESP32 Self-Learning Boat Autopilot

This is a MicroPython project for a self-learning, adaptive autopilot for small to medium-sized boats with hydraulic steering. The system uses an ESP32 microcontroller and a BNO055 9-axis sensor to hold a steady course, continuously adapting to the boat's unique characteristics and sea conditions.

---

## Features

- **Self-Learning:** The autopilot requires no manual calibration. It learns the boat's steering characteristics while in use.
- **Adaptive PI Control:** Uses a PI controller that compensates for constant drift from sources like crosswind or currents.
- **Dual-Core Architecture:** Utilizes both cores of the ESP32 processor for maximum stability:
    - **Core 0:** Dedicated to fast and continuous reading of the compass sensor.
    - **Core 1:** Runs the main logic for navigation, learning, and motor control.
- **Permanent Memory:** Learned calibration data is stored permanently in the ESP32's Non-Volatile Storage (NVS).

---

## Recommended Hardware

- **Microcontroller:** ESP32 DevKitC (or a similar standard ESP32 board).
- **Sensor:** Adafruit BNO055 Absolute Orientation Sensor.
- **Motor Driver:** A powerful H-Bridge that can handle the pump's current draw (e.g., BTS7960 or Infineon BTN99xx).
- **Power Supply:** A 12V to 5V DC-DC Buck Converter to provide stable power to the electronics.
- **Pump:** A 12V reversible hydraulic pump (e.g., Raymarine Type 1).

---

## Circuit Diagram

This diagram shows the complete wiring. The control electronics are powered by a switched 12V source (like the boat's ignition), while the high-current pump motor is powered directly from the battery. A common ground connection between both circuits is **absolutely critical** for the system to function.

```mermaid
graph TD
    subgraph Power Sources
        A[Main Battery 12V]
        B[Ignition Switched 12V]
    end

    subgraph Power Electronics
        C[BTS7960 H-Bridge Driver]
        F[Hydraulic Pump Motor]
    end

    subgraph Control Electronics
        D[ESP32-DevKitC]
        E[BNO055 Sensor]
        H[12V to 5V Buck Converter]
        G[Push Button]
    end

    %% High-Current Power Circuit
    A -- Direct 12V+ (HEAVY RED) --> C[B+]
    C[M+] -- Motor Wire (HEAVY BLUE) --> F[Terminal 1]
    C[M-] -- Motor Wire (HEAVY BLUE) --> F[Terminal 2]

    %% Switched Control Power Circuit
    B -- Switched 12V+ (RED) --> H[Vin+]
    H[Vout+] -- 5V+ (ORANGE) --> D[Vin]
    D[3.3V] -- 3.3V+ (YELLOW) --> E[Vin]

    %% Ground Connections (CRITICAL - ALL MUST CONNECT)
    A -- Main GND (HEAVY BLACK) --> C[B-]
    A -- Main GND (HEAVY BLACK) --> H[Vin-]
    H[Vout-] -- GND (BLACK) --> D[GND]
    D[GND] -- GND (BLACK) --> E[GND]
    D[GND] -- GND (BLACK) --> G[Pin 2]
    D[GND] -- GND (BLACK) --> C[GND]

    %% Control Signal Connections
    D[GPIO 26] -- DIR (BLUE) --> C[RPWM]
    D[GPIO 27] -- PWM (BLUE) --> C[LPWM]
    D[GPIO 33] -- BUTTON (GREEN) --> G[Pin 1]

    %% I2C Sensor Connections
    D[GPIO 22] -- SCL (PURPLE) --> E[SCL]
    D[GPIO 21] -- SDA (PURPLE) --> E[SDA]

    %% Styling
    linkStyle 0 stroke:red,stroke-width:4px
    linkStyle 1 stroke:#0077be,stroke-width:4px
    linkStyle 2 stroke:#0077be,stroke-width:4px
    linkStyle 3 stroke:red,stroke-width:2px
    linkStyle 4 stroke:orange,stroke-width:2px
    linkStyle 5 stroke:yellow,stroke-width:2px
    linkStyle 6 stroke:black,stroke-width:4px
    linkStyle 7 stroke:black,stroke-width:4px
    linkStyle 8 stroke:black,stroke-width:2px
    linkStyle 9 stroke:black,stroke-width:2px
    linkStyle 10 stroke:black,stroke-width:2px
    linkStyle 11 stroke:black,stroke-width:2px
    linkStyle 12 stroke:blue,stroke-width:2px
    linkStyle 13 stroke:blue,stroke-width:2px
    linkStyle 14 stroke:green,stroke-width:2px
    linkStyle 15 stroke:purple,stroke-width:2px
    linkStyle 16 stroke:purple,stroke-width:2px
```
