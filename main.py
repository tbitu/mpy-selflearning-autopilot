# main.py
# MicroPython Self-Learning and Adaptive Autopilot with PI-Control (Dual-Core)
# Enhanced for Outboard Engine Applications
#
# MAJOR IMPROVEMENTS FOR OUTBOARD ENGINES:
# 1. Engine Position Tracking - Handles mechanical hysteresis and deadband
# 2. Drift Compensation - Adapts to changing wind/current conditions  
# 3. Adaptive Learning Timing - Adjusts to sea state conditions
# 4. Engine Initialization - Determines starting engine position
# 5. Enhanced Safety Checks - Outboard-specific safety monitoring
# 6. Return Pulse Logic - Handles engine position management

import utime as time
import machine
import ujson
import _thread
from machine import Pin, I2C, NVS
from lib.bno055 import BNO055

# --- Classes for State Management ---
class State:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    INITIALIZING = "INITIALIZING"

class LearningState:
    IDLE = "IDLE"
    WAITING = "WAITING"
    MEASURING = "MEASURING"

class EnginePosition:
    def __init__(self):
        self.estimated_position = 0.0  # -100 to +100 (port to starboard)
        self.deadband = 3.0  # degrees of mechanical play
        self.max_deflection = 90.0  # maximum safe engine deflection
        self.position_confidence = 0.0  # 0-1, how confident we are in position
        
    def update_position(self, direction, pulse_ms, actual_turn_rate):
        """Update estimated engine position based on steering action"""
        # Estimate position change based on pulse duration and actual effect
        position_change = self.calculate_position_change(pulse_ms, actual_turn_rate)
        
        if direction == "starboard":
            self.estimated_position += position_change
        else:
            self.estimated_position -= position_change
            
        # Clamp to safe limits
        self.estimated_position = max(-self.max_deflection, 
                                    min(self.max_deflection, self.estimated_position))
        
        # Increase confidence as we learn
        self.position_confidence = min(1.0, self.position_confidence + 0.05)
        
    def calculate_position_change(self, pulse_ms, actual_turn_rate):
        """Estimate how much the engine position changed"""
        # Simple model: longer pulses = more position change
        # This should be calibrated based on actual engine characteristics
        base_change = (pulse_ms / 100.0) * 2.0  # Rough estimate
        
        # Adjust based on actual turn rate achieved
        if actual_turn_rate > 0:
            return base_change * min(2.0, actual_turn_rate / 1.0)
        return base_change
        
    def needs_return_pulse(self, desired_direction):
        """Check if engine needs to return toward center before next correction"""
        if self.position_confidence < 0.5:
            return False  # Not confident enough in position
            
        # If engine is deflected significantly in opposite direction
        if desired_direction == "starboard" and self.estimated_position < -20:
            return True
        if desired_direction == "port" and self.estimated_position > 20:
            return True
            
        return False
        
    def get_return_pulse_duration(self, desired_direction):
        """Calculate pulse needed to return engine toward center"""
        if desired_direction == "starboard":
            return int(abs(self.estimated_position) * 2)  # Rough estimate
        else:
            return int(abs(self.estimated_position) * 2)

class DriftCompensation:
    def __init__(self):
        self.drift_history = []  # Store heading errors over time
        self.baseline_adjustment = 0.0  # Current drift compensation
        self.max_history = 100  # Keep last 100 measurements
        self.drift_threshold = 1.5  # Degrees of consistent error to trigger adjustment
        
    def update_drift_data(self, heading_error):
        """Update drift compensation based on heading errors"""
        now = time.ticks_ms()
        self.drift_history.append((now, heading_error))
        
        # Keep only recent history
        if len(self.drift_history) > self.max_history:
            self.drift_history.pop(0)
            
        # Check for persistent drift every 20 measurements
        if len(self.drift_history) >= 20 and len(self.drift_history) % 20 == 0:
            self.detect_and_adjust_drift()
            
    def detect_and_adjust_drift(self):
        """Detect persistent drift and adjust baseline"""
        if len(self.drift_history) < 50:
            return
            
        # Analyze recent errors for persistent bias
        recent_errors = [error for _, error in self.drift_history[-30:]]
        avg_error = sum(recent_errors) / len(recent_errors)
        
        # Check if error is consistently in one direction
        same_sign_count = sum(1 for e in recent_errors if (e > 0) == (avg_error > 0))
        consistency = same_sign_count / len(recent_errors)
        
        # If consistently off course, adjust baseline
        if abs(avg_error) > self.drift_threshold and consistency > 0.7:
            adjustment = avg_error * 0.05  # Small gradual adjustment
            self.baseline_adjustment += adjustment
            
            # Limit total adjustment
            self.baseline_adjustment = max(-10.0, min(10.0, self.baseline_adjustment))
            
            print(f"Drift detected: avg_error={avg_error:.1f}, adjusting baseline by {adjustment:.2f}")
            
    def get_adjusted_target(self, original_target):
        """Get target heading adjusted for drift compensation"""
        adjusted = original_target + self.baseline_adjustment
        # Normalize to 0-360 range
        if adjusted < 0:
            adjusted += 360
        elif adjusted >= 360:
            adjusted -= 360
        return adjusted
        
    def estimate_sea_state(self):
        """Estimate sea conditions based on heading variance"""
        if len(self.drift_history) < 10:
            return 0.0
            
        recent_errors = [error for _, error in self.drift_history[-10:]]
        variance = sum((e - sum(recent_errors)/len(recent_errors))**2 for e in recent_errors) / len(recent_errors)
        return min(1.0, variance / 25.0)  # Normalize to 0-1

class Pins:
    PUMP_DIR = 26
    PUMP_PWM = 27
    NAVIGATE_BUTTON = 33
    SDA = 21
    SCL = 22
    LED = 19

class Navigation:
    CONTROL_LOOP_DELAY_MS = 2000
    SAVE_INTERVAL = 20
    PROPORTIONAL_GAIN = 0.6
    INTEGRAL_GAIN = 0.05
    INTEGRAL_MAX = 3.0

# --- Configuration ---

# PINS
PUMP_DIR_PIN = 26
PUMP_PWM_PIN = 27
NAVIGATE_BUTTON_PIN = 33
SDA_PIN = 21
SCL_PIN = 22
LED_PIN = 19 # Pin for the status LED

# LEARNING PARAMETERS
LEARNING_WAIT_SEC = 5      # Initial time to wait after a pulse before measuring
LEARNING_MEASURE_SEC = 8   # Initial time to measure the heading change
LEARNING_RATE = 0.2        # How much a new measurement affects the old one
MIN_HEADING_CHANGE = 0.5   # Minimum heading change to register as movement
MAX_WAIT_SEC = 10.0       # Maximum allowed wait time
MIN_WAIT_SEC = 2.0        # Minimum allowed wait time
MAX_MEASURE_SEC = 15.0    # Maximum allowed measure time
MIN_MEASURE_SEC = 3.0     # Minimum allowed measure time
TIMING_ADAPT_RATE = 0.3   # How quickly timing parameters adapt (0.3 = 30% new, 70% old)

# OUTBOARD ENGINE PARAMETERS
ENGINE_DEADBAND = 3.0      # Degrees of mechanical play in steering
ENGINE_MAX_DEFLECTION = 90.0  # Maximum safe engine deflection
INITIALIZATION_REQUIRED = True  # Whether engine position initialization is needed
ACTION_HISTORY_LIMIT = 15   # Maximum steering actions in 60 seconds (safety)

# NAVIGATION PARAMETERS
TARGET_HEADING = 180.0
SAVE_INTERVAL = 20         # Save calibration to memory after every 20 adjustments

# --- PI-CONTROL PARAMETERS ---
PROPORTIONAL_GAIN = 0.6  # (P) How strongly it reacts to the CURRENT error
INTEGRAL_GAIN = 0.05     # (I) Reduced since drift compensation handles persistent errors
INTEGRAL_MAX = 3.0       # Reduced max value to prevent conflicts with position tracking

# DEFAULT CALIBRATION DATA (a "best guess" to start with)
# This is turn rate (degrees/sec) for a given pulse (ms)
DEFAULT_CALIBRATION_DATA = {
    "starboard": {"100": 0.5, "200": 1.0, "400": 2.0, "600": 3.0, "900": 4.5},
    "port":      {"100": 0.5, "200": 1.0, "400": 2.0, "600": 3.0, "900": 4.5}
}

# --- Global Variables ---
state = State.IDLE
calibration_data = {}
current_heading = 0.0
save_counter = 0
integral_error = 0.0 # The "memory" for the PI controller

# Create new system components
engine_position = EnginePosition()
drift_compensation = DriftCompensation()

# Action history for safety monitoring
action_history = []  # Store timestamps of steering actions

# Adaptive timing variables
current_wait_time = LEARNING_WAIT_SEC
current_measure_time = LEARNING_MEASURE_SEC
learning_state = LearningState.IDLE  # For backward compatibility during refactoring
last_action_time = 0
heading_before_measure = 0.0
turn_start_time = 0  # For measuring response characteristics

# Turn rate tracking
heading_history = []
MAX_HISTORY = 10  # Store last 10 headings for turn rate calculation
last_turn_direction = ""   # Track last turning direction
turn_rate_threshold = 0.2  # degrees per second

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
                print("Sensor reinitialized after error")
            except Exception as init_error:
                print(f"Failed to reinitialize sensor: {init_error}")
                time.sleep(2)  # Longer delay before retry

# --- Core 1: Main Logic ---

def run_pump(direction, duration_ms):
    """Run the pump in the specified direction for the given duration.
    
    Args:
        direction (str): Either 'port' or 'starboard'
        duration_ms (int): Duration in milliseconds
    """
    global action_history
    
    if not enhanced_safety_check():
        print("Enhanced safety check failed, not running pump")
        return
        
    if direction not in ['port', 'starboard']:
        print(f"Invalid pump direction: {direction}")
        return
    if duration_ms <= 0 or duration_ms > 1000:  # Safety limit
        print(f"Invalid pump duration: {duration_ms}ms")
        return
    
    # Record action for safety monitoring
    now = time.ticks_ms()
    action_history.append(now)
    
    # Clean old entries (keep only last 60 seconds)
    action_history[:] = [t for t in action_history if time.ticks_diff(now, t) < 60000]
        
    print(f"Pump: Direction='{direction}', Duration={duration_ms}ms, Engine pos: {engine_position.estimated_position:.1f}")
    try:
        pump_dir.value(1 if direction == "starboard" else 0)
        pump_pwm.duty(1023)
        time.sleep(duration_ms / 1000)
    finally:
        pump_pwm.duty(0)  # Ensure pump stops even if there's an error

def save_calibration():
    """Save boat-specific learned parameters to persistent storage"""
    try:
        # Only save boat characteristics that persist across sessions
        learning_data = {
            'calibration_data': calibration_data,
            'boat_characteristics': {
                'base_learning_wait_sec': LEARNING_WAIT_SEC,
                'base_learning_measure_sec': LEARNING_MEASURE_SEC
            },
            'version': 1.0  # For future compatibility
        }
        
        nvs.set_blob('boat_data', ujson.dumps(learning_data))
        nvs.commit()
        print("Boat characteristics saved.")
    except Exception as e:
        print(f"Error saving boat data: {e}")

def load_calibration():
    """Load boat-specific learned parameters from persistent storage"""
    global calibration_data, LEARNING_WAIT_SEC, LEARNING_MEASURE_SEC
    
    try:
        # Try to load boat characteristics
        json_data = nvs.get_blob('boat_data')
        boat_data = ujson.loads(json_data)
        
        # Load calibration data (how the boat responds to steering)
        if ('calibration_data' in boat_data and 
            isinstance(boat_data['calibration_data'], dict) and 
            'starboard' in boat_data['calibration_data'] and 
            'port' in boat_data['calibration_data']):
            calibration_data = boat_data['calibration_data']
            print("Boat calibration data loaded from memory.")
        else:
            calibration_data = DEFAULT_CALIBRATION_DATA
            print("Invalid calibration data, using defaults.")
            
        # Load boat-specific timing characteristics
        if 'boat_characteristics' in boat_data:
            char_data = boat_data['boat_characteristics']
            if 'base_learning_wait_sec' in char_data:
                LEARNING_WAIT_SEC = char_data['base_learning_wait_sec']
            if 'base_learning_measure_sec' in char_data:
                LEARNING_MEASURE_SEC = char_data['base_learning_measure_sec']
            print(f"Boat timing characteristics loaded: wait={LEARNING_WAIT_SEC:.1f}s, measure={LEARNING_MEASURE_SEC:.1f}s")
            
    except (OSError, ValueError, TypeError, KeyError):
        # Fallback: try to load legacy format
        try:
            json_data = nvs.get_blob('cal_data')
            loaded_data = ujson.loads(json_data)
            
            if isinstance(loaded_data, dict) and 'starboard' in loaded_data and 'port' in loaded_data:
                calibration_data = loaded_data
                print("Legacy calibration data loaded.")
            else:
                calibration_data = DEFAULT_CALIBRATION_DATA
                print("Invalid legacy calibration data, using defaults.")
        except:
            calibration_data = DEFAULT_CALIBRATION_DATA
            print("No previous boat data found, using defaults.")

def needs_initial_learning():
    """Check if the system needs initial learning (no previous calibration)"""
    try:
        # Check for boat characteristics
        nvs.get_blob('boat_data')
        return False  # Boat data exists
    except (OSError, ValueError):
        try:
            # Check for legacy format
            nvs.get_blob('cal_data')
            return False  # Legacy calibration data exists
        except (OSError, ValueError):
            return True  # No calibration data found

def perform_initial_learning():
    """Perform initial learning sequence to establish basic calibration"""
    global calibration_data, state
    print("No previous calibration found. Starting initial learning sequence...")
    state = State.INITIALIZING
    
    try:
        # Initialize with minimal default values
        calibration_data = {
            "starboard": {"200": 0.5, "400": 1.0, "600": 1.5},
            "port": {"200": 0.5, "400": 1.0, "600": 1.5}
        }
        
        initial_heading = current_heading
        
        # Test starboard response
        print("Learning starboard response...")
        for pulse_ms in [200, 400, 600]:
            run_pump("starboard", pulse_ms)
            time.sleep(3)  # Wait for response
            
            heading_change = abs(get_heading_difference(initial_heading, current_heading))
            turn_rate = heading_change / 3.0  # 3 second observation
            
            if turn_rate > 0.1:  # Minimum detectable turn
                calibration_data["starboard"][str(pulse_ms)] = turn_rate
                print(f"Starboard {pulse_ms}ms: {turn_rate:.2f}°/s")
            
            time.sleep(2)  # Settle time
            
        # Return to initial heading roughly
        time.sleep(5)
        
        # Test port response  
        print("Learning port response...")
        for pulse_ms in [200, 400, 600]:
            run_pump("port", pulse_ms)
            time.sleep(3)  # Wait for response
            
            heading_change = abs(get_heading_difference(initial_heading, current_heading))
            turn_rate = heading_change / 3.0  # 3 second observation
            
            if turn_rate > 0.1:  # Minimum detectable turn
                calibration_data["port"][str(pulse_ms)] = turn_rate
                print(f"Port {pulse_ms}ms: {turn_rate:.2f}°/s")
            
            time.sleep(2)  # Settle time
            
        # Save initial calibration
        save_calibration()
        print("Initial learning complete. Basic calibration established.")
        
    except Exception as e:
        print(f"Error during initial learning: {e}")
        calibration_data = DEFAULT_CALIBRATION_DATA
    
    finally:
        state = State.IDLE

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
    if learning_state != LearningState.IDLE:
        return

    # Check if it's time for the next control loop iteration
    if time.ticks_diff(now, last_action_time) < Navigation.CONTROL_LOOP_DELAY_MS:
        return
    
    # Calculate current turn rate
    current_turn_rate = calculate_turn_rate()
    
    # If we're already turning in the desired direction, wait
    if abs(current_turn_rate) > turn_rate_threshold:
        # Get drift-compensated target
        adjusted_target = drift_compensation.get_adjusted_target(TARGET_HEADING)
        heading_error = get_heading_difference(current_heading, adjusted_target)
        if ((current_turn_rate > 0 and heading_error > 0) or 
            (current_turn_rate < 0 and heading_error < 0)):
            return  # Already turning in the right direction
            
    last_action_time = now

    # 1. FIND HEADING ERROR (with drift compensation)
    heading_now = current_heading
    adjusted_target = drift_compensation.get_adjusted_target(TARGET_HEADING)
    heading_error = get_heading_difference(heading_now, adjusted_target)
    
    # Update drift compensation data
    drift_compensation.update_drift_data(heading_error)
    
    # 2. UPDATE INTEGRAL MEMORY
    integral_error += heading_error
    
    # Reset integral if engine position changed significantly (reduces conflict with position tracking)
    if abs(engine_position.estimated_position) > 30:
        integral_error *= 0.5  # Reduce integral when engine is deflected
    
    # Clamp the integral to prevent "integral windup"
    if integral_error > Navigation.INTEGRAL_MAX: integral_error = Navigation.INTEGRAL_MAX
    if integral_error < -Navigation.INTEGRAL_MAX: integral_error = -Navigation.INTEGRAL_MAX
    
    print(f"Heading: {heading_now:.1f}, Target: {TARGET_HEADING:.1f} (adj: {adjusted_target:.1f}), Error: {heading_error:.1f}, Memory: {integral_error:.1f}")

    if abs(heading_error) < 1.0:
        # If we are on course, let the integral slowly "forget" to prevent over-correction
        integral_error *= 0.9
        return

    # 3. CALCULATE REQUIRED TURN RATE (P + I)
    required_turn_rate = calculate_required_turn(heading_error, integral_error)
    
    # Determine direction based on current turn rate and heading error
    if abs(current_turn_rate) > turn_rate_threshold:
        # If turning too fast, apply counter-rudder
        direction = "port" if current_turn_rate > 0 else "starboard"
        required_turn_rate = abs(current_turn_rate) * 0.5  # 50% counter-rudder
    else:
        direction = "starboard" if (heading_error * Navigation.PROPORTIONAL_GAIN + integral_error * Navigation.INTEGRAL_GAIN) > 0 else "port"
    
    # Check if engine needs return pulse due to position
    if engine_position.needs_return_pulse(direction):
        # Apply return pulse first
        return_direction = "port" if direction == "starboard" else "starboard"
        return_pulse = engine_position.get_return_pulse_duration(direction)
        return_pulse = min(300, max(100, return_pulse))  # Safety limits
        
        print(f"Applying return pulse: {return_direction} {return_pulse}ms")
        run_pump(return_direction, return_pulse)
        
        # Update engine position estimate
        engine_position.update_position(return_direction, return_pulse, 0.5)  # Rough estimate
        
        # Reset timer and return (wait for next cycle for main correction)
        last_action_time = time.ticks_ms()
        return
    
    best_pulse = find_best_pulse(direction, required_turn_rate)

    if best_pulse > 0:
        # Protect against division by zero
        calibrated_rate = calibration_data[direction][str(best_pulse)]
        if calibrated_rate > 0:
            overshoot_factor = required_turn_rate / calibrated_rate
        else:
            overshoot_factor = 1.0  # Fallback to normal pulse if calibration is zero
        pulse_to_run = int(best_pulse * overshoot_factor)
        
        # Store context for the learning part
        last_pulse_direction = direction
        last_pulse_ms = best_pulse

        run_pump(direction, pulse_to_run)
        
        # Initialize heading measurement for learning
        heading_before_measure = current_heading
        
        # Start the learning sequence with adaptive timing
        adaptive_learning_timing()
        learning_state = LearningState.WAITING
        last_action_time = time.ticks_ms() # Reset timer for next phase
        
        save_counter += 1
        if save_counter >= Navigation.SAVE_INTERVAL:
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
    actual_turn_rate = total_turn / current_measure_time
    
    # Update calibration data
    update_calibration_entry(direction, pulse_ms, actual_turn_rate)
    
    # Update engine position estimate
    engine_position.update_position(direction, pulse_ms, actual_turn_rate)
    
    print(f"Learning complete: {direction} {pulse_ms}ms -> {actual_turn_rate:.2f}°/s, engine pos: {engine_position.estimated_position:.1f}")

def safe_sensor_read():
    """Wrapper for sensor reading with error handling"""
    max_retries = 3
    for attempt in range(max_retries):
        try:
            heading, roll, pitch = bno.euler()
            return heading
        except Exception as e:
            print(f"Sensor read attempt {attempt + 1} failed: {e}")
            time.sleep(0.1)
    return None

def safety_check():
    """Perform safety checks before running the pump"""
    if not bno.calibrated():
        print("Warning: Sensor not calibrated")
        return False
    if abs(integral_error) >= Navigation.INTEGRAL_MAX * 0.9:
        print("Warning: Large integral error")
        return False
    return True

def enhanced_safety_check():
    """Enhanced safety checks for outboard engines"""
    # Basic safety checks first
    if not safety_check():
        return False
    
    # Check for engine position limits
    if abs(engine_position.estimated_position) > engine_position.max_deflection * 0.9:
        print(f"Warning: Engine near maximum deflection ({engine_position.estimated_position:.1f})")
        return False
    
    # Check for excessive correction frequency
    if len(action_history) > ACTION_HISTORY_LIMIT:
        print(f"Warning: Too many corrections ({len(action_history)} in 60s, limit: {ACTION_HISTORY_LIMIT})")
        return False
    
    # Check if state allows steering actions
    if state == State.INITIALIZING:
        return True  # Allow actions during initialization
    elif state != State.NAVIGATING:
        print("Warning: Not in navigation state")
        return False
    
    return True

def calculate_required_turn(heading_error, integral_error):
    """Calculate the required turn rate based on PI control"""
    p_term = abs(heading_error) * Navigation.PROPORTIONAL_GAIN
    i_term = abs(integral_error) * Navigation.INTEGRAL_GAIN
    return p_term + i_term

def initialize_engine_position():
    """Determine initial engine position through test movements"""
    global state
    print("Initializing engine position...")
    state = State.INITIALIZING
    
    try:
        # Record initial heading
        initial_heading = current_heading
        time.sleep(1)  # Ensure stable reading
        
        # Small test pulse to determine responsiveness
        print("Testing starboard response...")
        run_pump("starboard", 150)
        time.sleep(3)
        
        # Measure response
        heading_change_1 = get_heading_difference(initial_heading, current_heading)
        print(f"Initial starboard test: {heading_change_1:.2f} degrees")
        
        if abs(heading_change_1) < MIN_HEADING_CHANGE:
            # Engine might be off-center or need more force, try opposite direction
            print("Testing port response...")
            run_pump("port", 200)
            time.sleep(4)
            heading_change_2 = get_heading_difference(initial_heading, current_heading)
            print(f"Port test result: {heading_change_2:.2f} degrees")
            
            # Estimate initial position based on responses
            if abs(heading_change_2) > abs(heading_change_1):
                # Port was more effective, engine was probably starboard-biased
                engine_position.estimated_position = 15.0
            else:
                # Starboard was more effective or equal, engine probably centered
                engine_position.estimated_position = 0.0
        else:
            # Good response to starboard, engine probably near center
            engine_position.estimated_position = 0.0
            
        engine_position.position_confidence = 0.6  # Moderate confidence after initialization
        print(f"Engine initialization complete. Estimated position: {engine_position.estimated_position:.1f}")
        
        # Wait for boat to settle
        time.sleep(5)
        
    except Exception as e:
        print(f"Error during engine initialization: {e}")
        engine_position.estimated_position = 0.0
        engine_position.position_confidence = 0.3
    
    finally:
        state = State.IDLE

def adaptive_learning_timing():
    """Adjust learning parameters based on sea conditions"""
    global current_wait_time, current_measure_time
    
    sea_state = drift_compensation.estimate_sea_state()
    
    # Base timing
    base_wait = LEARNING_WAIT_SEC
    base_measure = LEARNING_MEASURE_SEC
    
    # Increase timing in rough conditions
    roughness_factor = 1.0 + (sea_state * 0.7)
    
    # Apply exponential moving average
    target_wait = base_wait * roughness_factor
    target_measure = base_measure * roughness_factor
    
    current_wait_time = (current_wait_time * 0.8) + (target_wait * 0.2)
    current_measure_time = (current_measure_time * 0.8) + (target_measure * 0.2)
    
    # Enforce limits
    current_wait_time = max(MIN_WAIT_SEC, min(MAX_WAIT_SEC, current_wait_time))
    current_measure_time = max(MIN_MEASURE_SEC, min(MAX_MEASURE_SEC, current_measure_time))

def find_best_pulse(direction, required_turn_rate):
    """Find the best pulse duration for the required turn rate"""
    pulses = calibration_data[direction]
    sorted_pulses = sorted([int(p) for p in pulses.keys()])
    for pulse_ms in sorted_pulses:
        if pulses[str(pulse_ms)] >= required_turn_rate:
            return pulse_ms
    return sorted_pulses[-1] if sorted_pulses and required_turn_rate > 0 else 0

def update_timing_parameters(response_time, settling_time):
    """Update timing parameters based on observed boat behavior"""
    global LEARNING_WAIT_SEC, LEARNING_MEASURE_SEC, PROPORTIONAL_GAIN, INTEGRAL_GAIN
    
    # Bound check response time
    response_time = max(MIN_WAIT_SEC, min(MAX_WAIT_SEC, response_time))
    settling_time = max(MIN_MEASURE_SEC, min(MAX_MEASURE_SEC, settling_time))
    
    # Update wait and measure times using exponential moving average
    LEARNING_WAIT_SEC = (LEARNING_WAIT_SEC * (1 - TIMING_ADAPT_RATE) + 
                        response_time * TIMING_ADAPT_RATE)
    LEARNING_MEASURE_SEC = (LEARNING_MEASURE_SEC * (1 - TIMING_ADAPT_RATE) + 
                          settling_time * TIMING_ADAPT_RATE)
    
    # Adjust PI gains based on response characteristics
    # Slower response = lower gains to prevent oscillation
    new_p_gain = PROPORTIONAL_GAIN * (5.0 / response_time)
    new_i_gain = INTEGRAL_GAIN * (5.0 / settling_time)
    
    # Bound the gains to safe ranges
    PROPORTIONAL_GAIN = max(0.2, min(0.8, new_p_gain))
    INTEGRAL_GAIN = max(0.05, min(0.15, new_i_gain))
    
    print(f"Updated timing - Wait: {LEARNING_WAIT_SEC:.1f}s, Measure: {LEARNING_MEASURE_SEC:.1f}s")
    print(f"Updated gains - P: {PROPORTIONAL_GAIN:.2f}, I: {INTEGRAL_GAIN:.2f}")
    
    # Save boat characteristics occasionally (every 10 updates to capture long-term learning)
    if not hasattr(update_timing_parameters, 'save_counter'):
        update_timing_parameters.save_counter = 0
    update_timing_parameters.save_counter += 1
    if update_timing_parameters.save_counter >= 10:
        save_calibration()
        update_timing_parameters.save_counter = 0

def calculate_turn_rate():
    """Calculate current turn rate based on recent heading changes.
    Positive values mean turning starboard, negative mean turning port."""
    global heading_history
    
    now = time.ticks_ms()
    heading_history.append((now, current_heading))
    if len(heading_history) > MAX_HISTORY:
        heading_history.pop(0)
    
    if len(heading_history) < 3:  # Need at least 3 points for stable calculation
        return 0.0
    
    # Use linear regression over multiple points for more stable turn rate
    time_diffs = []
    heading_diffs = []
    
    for i in range(1, len(heading_history)):
        time_diff = time.ticks_diff(heading_history[i][0], heading_history[i-1][0]) / 1000
        heading_diff = get_heading_difference(heading_history[i-1][1], heading_history[i][1])
        
        if time_diff > 0:
            time_diffs.append(time_diff)
            heading_diffs.append(heading_diff)
    
    if not time_diffs:
        return 0.0
        
    # Simple average of turn rates
    turn_rates = [h_diff / t_diff for h_diff, t_diff in zip(heading_diffs, time_diffs)]
    return sum(turn_rates) / len(turn_rates)

# --- Main Program (Runs on Core 1) ---
_thread.start_new_thread(sensor_loop, ())
print("Autopilot starting. Loading boat characteristics...")
load_calibration()

# Update current timing variables with loaded boat characteristics
current_wait_time = LEARNING_WAIT_SEC
current_measure_time = LEARNING_MEASURE_SEC

# Reset session-specific parameters (these change every fishing trip)
engine_position.estimated_position = 0.0       # Engine position unknown at startup
engine_position.position_confidence = 0.0      # No confidence in position
drift_compensation.baseline_adjustment = 0.0   # No drift compensation yet
drift_compensation.drift_history = []          # Clear drift history
integral_error = 0.0                           # Reset PI controller memory

print("Session-specific parameters reset for new fishing trip.")

# Check if initial learning is needed (for systems with no previous calibration)
if needs_initial_learning():
    print("No boat data found. Initial learning will be performed on first navigation start.")

# For non-blocking logic
last_button_check = 0
last_pulse_direction = ""
last_pulse_ms = 0
needs_initial_cal = needs_initial_learning()

while True:
    now = time.ticks_ms()

    # Handle button press (non-blocking)
    if time.ticks_diff(now, last_button_check) > 500: # Check every 500ms
        if not navigate_button.value():
            last_button_check = now # Debounce
            if state == State.NAVIGATING:
                state = State.IDLE
                learning_state = LearningState.IDLE # Reset learning
                led_pin.value(0) # Turn LED OFF
                print("Navigation stopped.")
            elif state == State.IDLE:
                # Check if initial learning is needed (for completely unknown systems)
                if needs_initial_cal:
                    print("Performing initial learning...")
                    perform_initial_learning()
                    needs_initial_cal = False
                
                # Check if engine initialization is needed
                if INITIALIZATION_REQUIRED and engine_position.position_confidence < 0.5:
                    print("Engine position unknown, initializing...")
                    initialize_engine_position()
                    
                TARGET_HEADING = current_heading # Grab the current heading
                integral_error = 0.0 # Reset memory when setting a new course
                drift_compensation.baseline_adjustment = 0.0  # Reset drift compensation
                state = State.NAVIGATING
                last_action_time = now # Start control loop timer
                led_pin.value(1) # Turn LED ON
                print(f"Navigation started. Holding new course: {TARGET_HEADING:.1f}")
                print(f"Engine position confidence: {engine_position.position_confidence:.1f}")
            elif state == State.INITIALIZING:
                print("Engine initialization in progress, please wait...")

    # --- Learning State Machine ---
    if learning_state == LearningState.WAITING:
        if time.ticks_diff(now, last_action_time) >= (current_wait_time * 1000):
            # Check if we have enough movement to start measuring
            current_change = abs(get_heading_difference(current_heading, heading_before_measure))
            if current_change >= MIN_HEADING_CHANGE:
                turn_start_time = now  # Record when the turn started
                heading_before_measure = current_heading
                learning_state = LearningState.MEASURING
                last_action_time = now
            elif time.ticks_diff(now, last_action_time) >= (MAX_WAIT_SEC * 1000):
                # If no movement after max wait time, abort learning cycle
                learning_state = LearningState.IDLE
                print("No significant movement detected, skipping measurement")
    
    elif learning_state == LearningState.MEASURING:
        if time.ticks_diff(now, last_action_time) >= (current_measure_time * 1000):
            # Calculate actual response characteristics
            response_time = time.ticks_diff(last_action_time, turn_start_time) / 1000
            settling_time = time.ticks_diff(now, turn_start_time) / 1000
            
            # Update timing parameters based on observed behavior
            update_timing_parameters(response_time, settling_time)
            
            # Complete the learning cycle
            update_after_learning(last_pulse_direction, last_pulse_ms)
            learning_state = LearningState.IDLE
            # Allow the navigation logic to run again immediately
            last_action_time = time.ticks_ms() - Navigation.CONTROL_LOOP_DELAY_MS 


    if state == State.NAVIGATING:
        run_navigation_cycle()
    elif state == State.INITIALIZING:
        # During initialization, just wait
        time.sleep(0.5)
    else:
        # Prevent busy-waiting when idle
        time.sleep(0.1)
    
    # The main loop delay is now handled inside handle_navigation_and_learning
    # and by the learning state machine.
    # A small delay to prevent CPU hogging when idle is still good.
    # The logic above handles this.
