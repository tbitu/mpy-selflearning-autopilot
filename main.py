# main.py
# MicroPython Self-Learning and Adaptive Autopilot with PI-Control (Dual-Core)

import utime as time
import machine
import ujson
import _thread
from machine import Pin, I2C, NVS
from lib.bno055 import BNO055

# --- Configuration ---

# PINS
PUMP_DIR_PIN = 26
PUMP_PWM_PIN = 27
NAVIGATE_BUTTON_PIN = 33
SDA_PIN = 21
SCL_PIN = 22
LED_PIN = 19 # Pin for the status LED

# LEARNING PARAMETERS
LEARNING_WAIT_SEC = 2      # Time in seconds to wait after a pulse before measuring
LEARNING_MEASURE_SEC = 3   # Time in seconds to measure the heading change
LEARNING_RATE = 0.2        # How much a new measurement affects the old one (0.1 = 10% new, 90% old)

# NAVIGATION PARAMETERS
TARGET_HEADING = 180.0
CONTROL_LOOP_DELAY_MS = 1000
SAVE_INTERVAL = 20         # Save calibration to memory after every 20 adjustments

# --- PI-CONTROL PARAMETERS ---
PROPORTIONAL_GAIN = 0.6  # (P) How strongly it reacts to the CURRENT error
INTEGRAL_GAIN = 0.1      # (I) How strongly it reacts to OLD, accumulated error
INTEGRAL_MAX = 5.0       # Max value for the "memory" to prevent it from running away

# DEFAULT CALIBRATION DATA (a "best guess" to start with)
# This is turn rate (degrees/sec) for a given pulse (ms)
DEFAULT_CALIBRATION_DATA = {
    "starboard": {"100": 0.5, "200": 1.0, "400": 2.0, "600": 3.0, "900": 4.5},
    "port":      {"100": 0.5, "200": 1.0, "400": 2.0, "600": 3.0, "900": 4.5}
}

# --- Global Variables ---
state = "IDLE"
calibration_data = {}
current_heading = 0.0
save_counter = 0
integral_error = 0.0 # The "memory" for the PI controller

# Non-blocking state variables
learning_state = "IDLE" # "IDLE", "WAITING", "MEASURING"
last_action_time = 0
heading_before_measure = 0.0

# --- Hardware Initialization ---
pump_dir = Pin(PUMP_DIR_PIN, Pin.OUT)
pump_pwm = machine.PWM(Pin(PUMP_PWM_PIN), freq=1000, duty=0)
navigate_button = Pin(NAVIGATE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
led_pin = Pin(LED_PIN, Pin.OUT) # Initialize LED pin
led_pin.value(0) # Ensure LED is off at startup
nvs = NVS("autopilot")

# Initialize I2C and BNO055
i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN))
bno = BNO055(i2c)

# --- Core 0: Sensor Thread ---

def sensor_loop():
    """
    This loop runs on Core 0.
    Its purpose is to continuously read the sensor as fast as possible
    and update the global 'current_heading' variable.
    """
    global current_heading
    print("Sensor thread started on Core 0.")
    while True:
        try:
            heading, roll, pitch = bno.euler()
            current_heading = heading
            time.sleep(0.05)  # Read the sensor approx. 20 times per second
        except Exception as e:
            print(f"Error in sensor thread: {e}")
            time.sleep(1)
            try:
                bno.init_sensor()  # Try to reinitialize on error
            except:
                pass

# --- Core 1: Main Logic ---

def run_pump(direction, duration_ms):
    """Run the pump in the specified direction for the given duration.
    
    Args:
        direction (str): Either 'port' or 'starboard'
        duration_ms (int): Duration in milliseconds
    """
    if direction not in ['port', 'starboard']:
        print(f"Invalid pump direction: {direction}")
        return
    if duration_ms <= 0 or duration_ms > 1000:  # Safety limit
        print(f"Invalid pump duration: {duration_ms}ms")
        return
        
    print(f"Pump: Direction='{direction}', Duration={duration_ms}ms")
    try:
        pump_dir.value(1 if direction == "starboard" else 0)
        pump_pwm.duty(1023)
        time.sleep(duration_ms / 1000)
    finally:
        pump_pwm.duty(0)  # Ensure pump stops even if there's an error

def save_calibration():
    try:
        nvs.set_blob('cal_data', ujson.dumps(calibration_data))
        nvs.commit()
        print("Calibration data saved.")
    except Exception as e:
        print(f"Error saving calibration: {e}")

def load_calibration():
    global calibration_data
    try:
        json_data = nvs.get_blob('cal_data')
        calibration_data = ujson.loads(json_data)
        print("Calibration data loaded from memory.")
    except (OSError, ValueError):
        calibration_data = DEFAULT_CALIBRATION_DATA
        print("Using default calibration data.")

def get_heading_difference(h1, h2):
    diff = h2 - h1
    if diff > 180: diff -= 360
    elif diff < -180: diff += 360
    return diff

def update_calibration_entry(direction, pulse_ms, new_turn_rate):
    global calibration_data
    pulse_key = str(pulse_ms)
    if pulse_key not in calibration_data[direction]:
        calibration_data[direction][pulse_key] = new_turn_rate
        return
    old_turn_rate = calibration_data[direction][pulse_key]
    # Weighted average: (old * (1-rate)) + (new * rate)
    updated_rate = (old_turn_rate * (1 - LEARNING_RATE)) + (new_turn_rate * LEARNING_RATE)
    calibration_data[direction][pulse_key] = updated_rate
    print(f"Updated calibration for {direction} {pulse_key}ms: {old_turn_rate:.2f} -> {updated_rate:.2f}")

def run_navigation_cycle():
    """
    This function runs one iteration of the navigation logic.
    It's called only when the state is "NAVIGATING" and we are not in a learning delay.
    """
    global save_counter, integral_error, learning_state, last_action_time, \
           last_pulse_direction, last_pulse_ms
    
    now = time.ticks_ms()

    # --- Navigation Logic (only if not in a learning-related delay) ---
    if learning_state != "IDLE":
        return

    # Check if it's time for the next control loop iteration
    if time.ticks_diff(now, last_action_time) < CONTROL_LOOP_DELAY_MS:
        return
    last_action_time = now

    # 1. FIND HEADING ERROR
    heading_now = current_heading
    heading_error = get_heading_difference(heading_now, TARGET_HEADING)
    
    # 2. UPDATE INTEGRAL MEMORY
    integral_error += heading_error
    # Clamp the integral to prevent "integral windup"
    if integral_error > INTEGRAL_MAX: integral_error = INTEGRAL_MAX
    if integral_error < -INTEGRAL_MAX: integral_error = -INTEGRAL_MAX
    
    print(f"Heading: {heading_now:.1f}, Target: {TARGET_HEADING:.1f}, Error: {heading_error:.1f}, Memory: {integral_error:.1f}")

    if abs(heading_error) < 1.0:
        # If we are on course, let the integral slowly "forget" to prevent over-correction
        integral_error *= 0.9
        return

    # 3. CALCULATE REQUIRED TURN RATE (P + I)
    p_term = abs(heading_error) * PROPORTIONAL_GAIN
    i_term = abs(integral_error) * INTEGRAL_GAIN
    required_turn_rate = p_term + i_term
    
    direction = "starboard" if (heading_error * PROPORTIONAL_GAIN + integral_error * INTEGRAL_GAIN) > 0 else "port"
    
    # The rest of the logic to find and run the pump is the same
    pulses = calibration_data[direction]
    sorted_pulses = sorted([int(p) for p in pulses.keys()])
    best_pulse = 0
    for pulse_ms in sorted_pulses:
        if pulses[str(pulse_ms)] >= required_turn_rate:
            best_pulse = pulse_ms
            break
    if best_pulse == 0 and required_turn_rate > 0:
        best_pulse = sorted_pulses[-1]

    if best_pulse > 0:
        overshoot_factor = required_turn_rate / pulses[str(best_pulse)]
        pulse_to_run = int(best_pulse * overshoot_factor)
        
        # Store context for the learning part
        # This is a simplified way. A better way would be a class or a dict.
        last_pulse_direction = direction
        last_pulse_ms = best_pulse

        run_pump(direction, pulse_to_run)
        
        # Start the learning sequence
        learning_state = "WAITING"
        last_action_time = time.ticks_ms() # Reset timer for next phase
        
        # The rest of the learning logic is now in the state machine
        
        save_counter += 1
        if save_counter >= SAVE_INTERVAL:
            save_calibration()
            save_counter = 0

def update_after_learning(direction, pulse_ms):
    """
    This function is called after the measurement phase of learning is complete.
    Measures the actual turn rate achieved and updates the calibration data.
    """
    global heading_before_measure
    heading_after_measure = current_heading
    total_turn = abs(get_heading_difference(heading_before_measure, heading_after_measure))
    actual_turn_rate = total_turn / LEARNING_MEASURE_SEC
    update_calibration_entry(direction, pulse_ms, actual_turn_rate)


# --- Main Program (Runs on Core 1) ---
_thread.start_new_thread(sensor_loop, ())
print("Autopilot starting. Loading calibration...")
load_calibration()

# For non-blocking logic
last_button_check = 0
last_pulse_direction = ""
last_pulse_ms = 0

while True:
    now = time.ticks_ms()

    # Handle button press (non-blocking)
    if time.ticks_diff(now, last_button_check) > 500: # Check every 500ms
        if not navigate_button.value():
            last_button_check = now # Debounce
            if state == "NAVIGATING":
                state = "IDLE"
                learning_state = "IDLE" # Reset learning
                led_pin.value(0) # Turn LED OFF
                print("Navigation stopped.")
            elif state == "IDLE":
                TARGET_HEADING = current_heading # Grab the current heading
                integral_error = 0.0 # Reset memory when setting a new course
                state = "NAVIGATING"
                last_action_time = now # Start control loop timer
                led_pin.value(1) # Turn LED ON
                print(f"Navigation started. Holding new course: {TARGET_HEADING:.1f}")

    # --- Learning State Machine ---
    if learning_state == "WAITING":
        if time.ticks_diff(now, last_action_time) >= (LEARNING_WAIT_SEC * 1000):
            heading_before_measure = current_heading
            learning_state = "MEASURING"
            last_action_time = now
    
    elif learning_state == "MEASURING":
        if time.ticks_diff(now, last_action_time) >= (LEARNING_MEASURE_SEC * 1000):
            update_after_learning(last_pulse_direction, last_pulse_ms)
            learning_state = "IDLE"
            # Allow the navigation logic to run again immediately
            last_action_time = time.ticks_ms() - CONTROL_LOOP_DELAY_MS 


    if state == "NAVIGATING":
        run_navigation_cycle()
    else:
        # Prevent busy-waiting when idle
        time.sleep(0.1)
    
    # The main loop delay is now handled inside handle_navigation_and_learning
    # and by the learning state machine.
    # A small delay to prevent CPU hogging when idle is still good.
    # The logic above handles this.
