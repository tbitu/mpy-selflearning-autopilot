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
    %% Define Subgraphs for better organization
    subgraph Power Sources
        A[Boat's 12V Battery]
        B[Ignition Switched 12V]
    end

    subgraph High-Current Pump Circuit
        C[Motor Driver e.g., BTS7960 H-Bridge]
        F[12V Hydraulic Pump Motor]
    end

    subgraph Control Electronics
        subgraph ESP32-DevKitC
            direction LR
            D_VIN[Vin]
            D_GND[GND]
            D_3V3[3.3V]
            D_GPIO21[GPIO 21 / I2C SDA]
            D_GPIO22[GPIO 22 / I2C SCL]
            D_GPIO26[GPIO 26 / Pump DIR]
            D_GPIO27[GPIO 27 / Pump PWM]
            D_GPIO33[GPIO 33 / Button]
        end
        E[BNO055 Sensor]
        H[12V to 5V DC-DC Buck Converter]
        G[Push Button]
    end

    %% Central Ground Bus
    subgraph Ground Connections
        GND[GND BUS]
    end
    style GND fill:#333,stroke:#fff,stroke-width:2px,color:#fff

    %% --- Power Connections ---
    A -- "Direct 12V+ (HEAVY GAUGE RED)" --> C[B+]
    B -- "Switched 12V+ (RED)" --> H[Input +]
    H -- "5V+ Output (ORANGE)" --> D_VIN
    D_3V3 -- "3.3V+ (YELLOW)" --> E[Vin]

    %% --- Ground Connections (All grounds connect to the central bus) ---
    A -- "GND (HEAVY GAUGE BLACK)" --> GND
    C[B-] -- "GND (BLACK)" --> GND
    H[Input -] -- "GND (BLACK)" --> GND
    H[Output -] -- "GND (BLACK)" --> GND
    D_GND -- "GND (BLACK)" --> GND
    E[GND] -- "GND (BLACK)" --> GND
    G[Pin 2] -- "GND (BLACK)" --> GND

    %% --- Control Signal Connections ---
    D_GPIO26 -- "Direction Signal (BLUE)" --> C[RPWM/DIR]
    D_GPIO27 -- "PWM Signal (BLUE)" --> C[LPWM/PWM]
    D_GPIO33 -- "Button Signal (GREEN)" --> G[Pin 1]

    %% --- I2C Sensor Connections ---
    D_GPIO22 -- "SCL (PURPLE)" --> E[SCL]
    D_GPIO21 -- "SDA (PURPLE)" --> E[SDA]

    %% --- Motor Output Connections ---
    C[M+] -- "Motor Wire (HEAVY GAUGE BLUE)" --> F[+]
    C[M-] -- "Motor Wire (HEAVY GAUGE BLUE)" --> F[-]

    %% --- Styling and Comments ---
    classDef esp32 fill:#00979D,stroke:#333,stroke-width:2px,color:white;
    class D_VIN,D_GND,D_3V3,D_GPIO21,D_GPIO22,D_GPIO26,D_GPIO27,D_GPIO33 esp32;

    classDef sensor fill:#7952B3,stroke:#333,stroke-width:2px,color:white;
    class E sensor;

    classDef motorDriver fill:#DC3545,stroke:#333,stroke-width:2px,color:white;
    class C motorDriver;

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
