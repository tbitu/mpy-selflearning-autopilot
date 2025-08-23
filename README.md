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
        A[Boat's 12V Battery]
        B[Ignition Switched 12V]
    end

    subgraph High-Current Circuit
        C[BTS7960 H-Bridge Driver]
        F[Hydraulic Pump Motor]
    end

    subgraph Control Circuit
        subgraph ESP32-DevKitC
            D_VIN[Vin]
            D_GND[GND]
            D_3V3[3.3V]
            D_GPIO21[GPIO 21 / SDA]
            D_GPIO22[GPIO 22 / SCL]
            D_GPIO26[GPIO 26 / DIR]
            D_GPIO27[GPIO 27 / PWM]
            D_GPIO33[GPIO 33 / BTN]
        end
        E[BNO055 Sensor]
        H[12V to 5V Buck Converter]
        G[Push Button]
    end

    %% Ground Bus (All grounds connect here)
    subgraph GND Bus
        GND((GND))
    end
    style GND fill:#333,stroke:#fff

    %% Power Connections
    A -- Direct 12V+ (HEAVY RED) --> C[B+]
    B -- Switched 12V+ (RED) --> H[Vin+]
    H -- 5V+ (ORANGE) --> D_VIN
    D_3V3 -- 3.3V+ (YELLOW) --> E[Vin]

    %% Ground Connections
    A -- GND (HEAVY BLACK) --> GND
    C[B-] -- GND (BLACK) --> GND
    H[Vin-] -- GND (BLACK) --> GND
    H[Vout-] -- GND (BLACK) --> GND
    D_GND -- GND (BLACK) --> GND
    E[GND] -- GND (BLACK) --> GND
    G[Pin 2] -- GND (BLACK) --> GND

    %% Control Signals
    D_GPIO26 -- DIR (BLUE) --> C[RPWM]
    D_GPIO27 -- PWM (BLUE) --> C[LPWM]
    D_GPIO33 -- BTN (GREEN) --> G[Pin 1]

    %% I2C Sensor Signals
    D_GPIO22 -- SCL (PURPLE) --> E[SCL]
    D_GPIO21 -- SDA (PURPLE) --> E[SDA]

    %% Motor Output
    C[M+] -- Motor Wire (HEAVY BLUE) --> F[+]
    C[M-] -- Motor Wire (HEAVY BLUE) --> F[-]

    %% Styling
    linkStyle 0 stroke:red,stroke-width:4px
    linkStyle 1 stroke:red,stroke-width:2px
    linkStyle 2 stroke:orange,stroke-width:2px
    linkStyle 3 stroke:yellow,stroke-width:2px
    linkStyle 4 stroke:black,stroke-width:4px
    linkStyle 5 stroke:black,stroke-width:2px
    linkStyle 6 stroke:black,stroke-width:2px
    linkStyle 7 stroke:black,stroke-width:2px
    linkStyle 8 stroke:black,stroke-width:2px
    linkStyle 9 stroke:black,stroke-width:2px
    linkStyle 10 stroke:black,stroke-width:2px
    linkStyle 11 stroke:blue,stroke-width:2px
    linkStyle 12 stroke:blue,stroke-width:2px
    linkStyle 13 stroke:green,stroke-width:2px
    linkStyle 14 stroke:purple,stroke-width:2px
    linkStyle 15 stroke:purple,stroke-width:2px
    linkStyle 16 stroke:#0077be,stroke-width:4px
    linkStyle 17 stroke:#0077be,stroke-width:4px
```
