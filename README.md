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
    subgraph PowerSystem [Power System]
        direction LR
        A[Boat's 12V Battery]
        B[Ignition Switched 12V]
        H[12V to 5V Buck Converter]
    end

    subgraph HighCurrentCircuit [High-Current Pump Circuit]
        C[Motor Driver e.g., BTS7960]
        F[12V Hydraulic Pump]
    end

    subgraph ControlCircuit [Control Circuit]
        subgraph ESP32 [ESP32 DevKitC]
            direction LR
            D_VIN[Vin]
            D_GND[GND]
            D_3V3[3.3V]
            D_GPIO21[GPIO 21]
            D_GPIO22[GPIO 22]
            D_GPIO26[GPIO 26]
            D_GPIO27[GPIO 27]
            D_GPIO33[GPIO 33]
        end
        E[BNO055 Sensor]
        G[Navigate Button]
    end

    %% Central Ground Bus - CRITICAL
    subgraph GroundBus [Ground Bus - All Grounds Connect Here]
        GND[COMMON GND]
    end
    style GND fill:#333,stroke:#fff,stroke-width:2px,color:#fff

    %% --- POWER WIRING ---
    B -- "Switched 12V+" --> H[Input +]
    H -- "5V+ Output" --> D_VIN[Vin]
    D_3V3 -- "3.3V+" --> E[Vin]
    A -- "Direct 12V+ (Heavy Gauge)" --> C[B+]

    %% --- GROUND WIRING ---
    A -- "GND (Heavy Gauge)" --> GND
    H[Input -] -- "GND" --> GND
    H[Output -] -- "GND" --> GND
    D_GND -- "GND" --> GND
    E[GND] -- "GND" --> GND
    G[Pin 2] -- "GND" --> GND
    C[B-] -- "GND" --> GND

    %% --- PUMP CONTROL SIGNALS (from main.py) ---
    D_GPIO26 -- "PUMP_DIR_PIN" --> C[RPWM/DIR]
    D_GPIO27 -- "PUMP_PWM_PIN" --> C[LPWM/PWM]

    %% --- BUTTON WIRING (from main.py) ---
    D_GPIO33 -- "NAVIGATE_BUTTON_PIN" --> G[Pin 1]

    %% --- I2C SENSOR WIRING (from main.py) ---
    D_GPIO22 -- "SCL_PIN" --> E[SCL]
    D_GPIO21 -- "SDA_PIN" --> E[SDA]

    %% --- MOTOR OUTPUT WIRING ---
    C[M+] -- "Motor Wire (Heavy Gauge)" --> F[+]
    C[M-] -- "Motor Wire (Heavy Gauge)" --> F[-]

    %% --- Styling ---
    linkStyle 3 stroke:red,stroke-width:4px
    linkStyle 0,1,2 stroke:orange,stroke-width:2px

    linkStyle 4,5,6,7,8,9,10 stroke:black,stroke-width:2px

    linkStyle 11,12 stroke:blue,stroke-width:2px
    linkStyle 13 stroke:green,stroke-width:2px
    linkStyle 14,15 stroke:purple,stroke-width:2px

    linkStyle 16,17 stroke:#0077be,stroke-width:4px
```
