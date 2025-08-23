# ESP32 Self-Learning Boat Autopilot

This is a MicroPython project for a self-learning, adaptive autopilot for small to medium-sized boats with hydraulic steering. The system uses an ESP32 microcontroller and a BNO055 9-axis sensor to hold a steady course, continuously adapting to the boat's unique characteristics and sea conditions.

---

## Features

* **Self-Learning:** The autopilot requires no manual calibration. It learns the boat's steering characteristics while in use.
* **Adaptive PI Control:** Uses a PI controller that compensates for constant drift from sources like crosswind or currents.
* **Dual-Core Architecture:** Utilizes both cores of the ESP32 processor for maximum stability:
    * **Core 0:** Dedicated to fast and continuous reading of the compass sensor.
    * **Core 1:** Runs the main logic for navigation, learning, and motor control.
* **Permanent Memory:** Learned calibration data is stored permanently in the ESP32's Non-Volatile Storage (NVS).

---

## Recommended Hardware

* **Microcontroller:** ESP32 DevKitC (or a similar standard ESP32 board).
* **Sensor:** Adafruit BNO055 Absolute Orientation Sensor.
* **Motor Driver:** A powerful H-Bridge that can handle the pump's current draw (e.g., based on BTS7960 or Infineon BTN99xx).
* **Power Supply:** A 12V to 5V DC-DC Buck Converter to provide stable power to the electronics.
* **Pump:** A 12V reversible hydraulic pump (e.g., Raymarine Type 1).

---

## Setup and Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/tbitu/mpy-selflearning-autopilot.git
    ```

2.  **Flash MicroPython to the ESP32:** Follow the official instructions at [micropython.org](https://micropython.org/download/esp32/).

3.  **Download the BNO055 library:** Download the `bno055.py` file from the link above and place it in the `lib/` directory of the project.

4.  **Upload files to the ESP32:** Use a tool like `mpremote` to upload `main.py` and the entire `lib` folder to the ESP32 board.
    ```bash
    # Copy the library folder
    mpremote cp -r lib/ :

    # Copy the main program
    mpremote cp main.py :
    ```

5.  **Connect the hardware:** Wire the components according to a suitable circuit diagram. Ensure all components share a common ground.

6.  **Start the autopilot:** Reboot the ESP32 board. The system will load the default calibration and is ready to use. Press the navigation button to engage the autopilot and hold the current course.
