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
- **Motor Driver:** A powerful H-Bridge that can handle the pump's current draw (e.g., based on BTS7960 or Infineon BTN99xx).
- **Power Supply:** A 12V to 5V DC-DC Buck Converter to provide stable power to the electronics.
- **Pump:** A 12V reversible hydraulic pump (e.g., Raymarine Type 1).

---

## Circuit Diagram

This diagram shows how to wire all the components. **Note:** The control pin names on H-Bridges can vary. `L_EN`/`R_EN` or `RPWM`/`LPWM` are common. Connect them to the ESP32 pins as defined in `main.py`.

```mermaid
graph TD
    subgraph Power Source
        A[Boat's 12V Battery]
    end

    subgraph Power Distribution
        B[12V to 5V Buck Converter]
        C[H-Bridge Motor Driver e.g. BTS7960]
    end

    subgraph Control Unit
        D[ESP32 DevKitC]
    end

    subgraph Components
        E[BNO055 Sensor]
        F[Hydraulic Pump Motor]
        G[Push Button]
    end

    %% Power Connections
    A -- 12V+ (RED) --> B[Vin]
    A -- 12V+ (RED) --> C[VCC / B+]
    B -- 5V+ (ORANGE) --> D[Vin]
    D -- 3.3V (YELLOW) --> E[Vin]

    %% Ground Connections (CRITICAL - ALL MUST CONNECT)
    A -- GND (BLACK) --> B[GND In]
    B[GND Out] -- GND (BLACK) --> D[GND]
    D[GND] -- GND (BLACK) --> E[GND]
    D[GND] -- GND (BLACK) --> G[Pin 2]
    A -- GND (BLACK) --> C[GND / B-]

    %% Control Signal Connections
    D[GPIO 26] -- DIR (BLUE) --> C[L_EN / RPWM]
    D[GPIO 27] -- PWM (BLUE) --> C[R_EN / LPWM]
    D[GPIO 33] -- BUTTON (GREEN) --> G[Pin 1]

    %% I2C Sensor Connections
    D[GPIO 22 SCL] -- SCL (PURPLE) --> E[SCL]
    D[GPIO 21 SDA] -- SDA (PURPLE) --> E[SDA]

    %% High-Power Motor Output
    C[OUT L / M-] -- MOTOR (HEAVY BLUE) --> F[Terminal 1]
    C[OUT R / M+] -- MOTOR (HEAVY BLUE) --> F[Terminal 2]

    %% Styling
    style A fill:#f9f,stroke:#333,stroke-width:2px
    style F fill:#f9f,stroke:#333,stroke-width:2px
    linkStyle 0 stroke:red,stroke-width:2px
    linkStyle 1 stroke:red,stroke-width:2px
    linkStyle 2 stroke:orange,stroke-width:2px
    linkStyle 3 stroke:yellow,stroke-width:2px
    linkStyle 4 stroke:black,stroke-width:3px
    linkStyle 5 stroke:black,stroke-width:3px
    linkStyle 6 stroke:black,stroke-width:3px
    linkStyle 7 stroke:black,stroke-width:3px
    linkStyle 8 stroke:black,stroke-width:3px
    linkStyle 9 stroke:blue,stroke-width:2px
    linkStyle 10 stroke:blue,stroke-width:2px
    linkStyle 11 stroke:green,stroke-width:2px
    linkStyle 12 stroke:purple,stroke-width:2px
    linkStyle 13 stroke:purple,stroke-width:2px
    linkStyle 14 stroke:#0077be,stroke-width:4px
    linkStyle 15 stroke:#0077be,stroke-width:4px
```
