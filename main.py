# main.py
# MicroPython Self-Learning and Adaptive Autopilot
# Refactored for Outboard Trolling with Continuous Control Loop

import utime as time
import machine
import ujson
import _thread
import math
from machine import Pin, I2C, NVS

# Import custom libraries
from lib.bno055 import BNO055
from lib.motor import MotorController
from lib.pid import PID
from lib.gps_stub import GPSStub
from lib.autotuner import AutoTuner

# --- Constants & Configuration ---
# Pins
PUMP_DIR_PIN = 26
PUMP_PWM_PIN = 27
NAVIGATE_BUTTON_PIN = 33
SDA_PIN = 21
SCL_PIN = 22
LED_PIN = 19

# Control Loop
LOOP_FREQ = 10  # Hz
LOOP_DELAY_MS = 1000 // LOOP_FREQ

# Steering Parameters
MAX_RUDDER_POWER = 1023
MIN_RUDDER_POWER = 300  # Minimum power to move the motor
MAX_PULSE_MS = 500      # Max pulse duration per loop cycle (safety)

# Default PID Gains (will be tuned/loaded)
DEFAULT_KP = 1.0
DEFAULT_KI = 0.05
DEFAULT_KD = 0.5

# --- State Management ---
class State:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    CALIBRATING = "CALIBRATING"

# --- Global Objects ---
state = State.IDLE
target_heading = 0.0
current_heading = 0.0
current_roll = 0.0
current_pitch = 0.0
sea_state_variance = 0.0

# Hardware
led = Pin(LED_PIN, Pin.OUT)
button = Pin(NAVIGATE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN))
bno = BNO055(i2c)
nvs = NVS("autopilot")

# Controllers
motor = MotorController(PUMP_PWM_PIN, PUMP_DIR_PIN)
pid = PID(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, output_limits=(-100, 100))
gps = GPSStub()
autotuner = AutoTuner(pid)

# --- Helper Functions ---

def get_heading_difference(target, current):
    diff = target - current
    if diff > 180: diff -= 360
    elif diff < -180: diff += 360
    return diff

def quaternion_to_euler(q):
    """Convert quaternion (w, x, y, z) to euler (heading, roll, pitch)"""
    # BNO055 returns (w, x, y, z) scaled by 1/(1<<14)
    # But the driver might return them differently. 
    # The BNO055_BASE.quaternion lambda returns tuple (w, x, y, z)
    
    w, x, y, z = q
    
    # Yaw (Heading)
    sqw = w * w
    sqx = x * x
    sqy = y * y
    sqz = z * z
    
    # Heading calculation (standard conversion)
    t0 = 2.0 * (w * z + x * y)
    t1 = 1.0 - 2.0 * (y * y + z * z)
    heading = math.atan2(t0, t1)
    heading = math.degrees(heading)
    if heading < 0: heading += 360
    
    # Roll
    t2 = 2.0 * (w * x - y * z)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    roll = math.degrees(math.asin(t2))
    
    # Pitch
    t3 = 2.0 * (w * y + z * x)
    t4 = 1.0 - 2.0 * (x * x + y * y)
    pitch = math.degrees(math.atan2(t3, t4))
    
    return heading, roll, pitch

def save_settings():
    """Save PID gains and other settings to NVS"""
    try:
        settings = {
            'kp': pid.kp,
            'ki': pid.ki,
            'kd': pid.kd
        }
        nvs.set_blob('settings', ujson.dumps(settings))
        nvs.commit()
        print("Settings saved.")
    except Exception as e:
        print(f"Error saving settings: {e}")

def load_settings():
    """Load settings from NVS"""
    try:
        buf = bytearray(1024) # Allocate buffer
        length = nvs.get_blob('settings', buf)
        if length > 0:
            data = ujson.loads(buf[:length])
            pid.kp = data.get('kp', DEFAULT_KP)
            pid.ki = data.get('ki', DEFAULT_KI)
            pid.kd = data.get('kd', DEFAULT_KD)
            print(f"Loaded PID: P={pid.kp}, I={pid.ki}, D={pid.kd}")
    except Exception as e:
        print(f"Using default settings. ({e})")

# --- Core 0: Sensor Loop ---
def sensor_loop():
    global current_heading, current_roll, current_pitch, sea_state_variance
    
    print("Sensor thread started.")
    history = []
    
    while True:
        try:
            # Read Quaternion for stability
            q = bno.quaternion()
            h, r, p = quaternion_to_euler(q)
            
            current_heading = h
            current_roll = r
            current_pitch = p
            
            # Calculate sea state (variance of roll)
            history.append(r)
            if len(history) > 20: history.pop(0)
            if len(history) > 1:
                avg = sum(history) / len(history)
                var = sum((x - avg)**2 for x in history) / len(history)
                sea_state_variance = var
                
            time.sleep_ms(50) # 20Hz update
            
        except Exception as e:
            print(f"Sensor Error: {e}")
            time.sleep_ms(100)

# --- Core 1: Main Control Loop ---
def main():
    global state, target_heading
    
    # Initialize
    led.value(0)
    load_settings()
    
    # Start Sensor Thread
    _thread.start_new_thread(sensor_loop, ())
    
    print("Autopilot Ready. Press button to engage.")
    
    last_loop_time = time.ticks_ms()
    last_button_time = 0
    last_save_time = time.ticks_ms()
    
    while True:
        now = time.ticks_ms()
        
        # 1. Button Handling (Debounced)
        if button.value() == 0:
            if time.ticks_diff(now, last_button_time) > 500:
                last_button_time = now
                if state == State.IDLE:
                    state = State.NAVIGATING
                    target_heading = current_heading
                    pid.reset()
                    pid.setpoint = target_heading
                    led.value(1)
                    print(f"ENGAGED. Target: {target_heading:.1f}")
                else:
                    state = State.IDLE
                    motor.stop()
                    led.value(0)
                    save_settings() # Save on disengage
                    print("DISENGAGED.")
        
        # 2. Control Logic
        if state == State.NAVIGATING:
            # Update GPS (Stub)
            gps.update()
            
            # Calculate Error
            # TODO: Fuse GPS COG here if available
            
            # PID Update
            # Input: current_heading, Setpoint: target_heading
            # The PID class handles the wrap-around logic internally if we pass error, 
            # but our PID class expects measurement and setpoint.
            # We need to be careful with wrap-around passed to PID.
            # Let's pass 0 as setpoint and 'error' as measurement to simplify wrap-around handling
            # or rely on PID's internal wrap-around logic if we implemented it.
            # Checking PID class... yes, it handles wrap-around.
            
            pid.setpoint = target_heading
            output = pid.update(current_heading)
            
            # AutoTuner Update (Learns from boat response)
            autotuner.update(current_heading, output)
            
            # Periodic Save (every 60 seconds)
            if time.ticks_diff(now, last_save_time) > 60000:
                save_settings()
                last_save_time = now
            
            # Output is -100 to 100 (representing % of effort)
            
            # Convert to Motor Control
            # Strategy: "Time Proportional Control" within the loop period?
            # Or PWM Duty Cycle? 
            # Since we have a hydraulic pump, PWM duty cycle is better for smooth control.
            # But many pumps need a minimum duty to start.
            
            power = abs(output)
            direction = "starboard" if output > 0 else "port"
            
            if power < 5: # Deadband
                motor.stop()
            else:
                # Map 0-100 to MIN_POWER-MAX_POWER
                pwm_val = int(MIN_RUDDER_POWER + (power/100.0) * (MAX_RUDDER_POWER - MIN_RUDDER_POWER))
                pwm_val = min(MAX_RUDDER_POWER, pwm_val)
                
                # For hydraulic pumps, we might want to pulse it if the loop is fast.
                # But continuous PWM is smoother.
                motor.run(direction, LOOP_DELAY_MS + 10, pwm_val) 
                # Duration is slightly longer than loop to keep it continuous
        
        # 3. Motor Safety Update
        motor.update()
        
        # 4. Loop Timing
        elapsed = time.ticks_diff(time.ticks_ms(), now)
        delay = LOOP_DELAY_MS - elapsed
        if delay > 0:
            time.sleep_ms(delay)
            
if __name__ == "__main__":
    main()
