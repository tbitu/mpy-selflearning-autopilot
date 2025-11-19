import utime as time
import math

class AutoTuner:
    def __init__(self, pid_controller):
        self.pid = pid_controller
        self.last_heading = 0
        self.last_time = time.ticks_ms()
        self.samples = 0
        self.sum_responsiveness = 0
        
        # "Target" responsiveness: We assume a "standard" boat turns 
        # approx 5 degrees/sec at 100% power.
        # If the boat turns faster, we lower gains. If slower, we raise gains.
        # This constant acts as the "baseline" for Kp=1.0
        self.BASELINE_RESPONSIVENESS = 5.0 / 100.0 # 0.05 deg/s per % power
        
    def update(self, current_heading, motor_output):
        """
        Update the tuner with current state.
        motor_output: -100 to 100
        """
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000.0
        
        if dt < 0.1: return # Too fast to measure meaningful rate change
        
        # Calculate turn rate
        diff = current_heading - self.last_heading
        if diff > 180: diff -= 360
        elif diff < -180: diff += 360
        
        turn_rate = diff / dt # deg/s
        
        # Only analyze if we are applying significant power and turning
        # This filters out noise and idle times
        if abs(motor_output) > 20 and abs(turn_rate) > 0.2:
            # Responsiveness = deg/s / %power
            # Example: 2 deg/s / 50% = 0.04
            # We use absolute values because negative power should cause negative turn
            resp = abs(turn_rate) / abs(motor_output)
            
            self.sum_responsiveness += resp
            self.samples += 1
            
            # Update gains every 20 valid samples (approx 2-5 seconds of turning)
            if self.samples >= 20:
                avg_resp = self.sum_responsiveness / self.samples
                self.tune_pid(avg_resp)
                self.sum_responsiveness = 0
                self.samples = 0
                
        self.last_heading = current_heading
        self.last_time = now
        
    def tune_pid(self, measured_responsiveness):
        if measured_responsiveness <= 0.0001: return
        
        # Calculate the ideal Kp based on the ratio of Baseline vs Actual responsiveness
        # If Actual is 2x Baseline (very sensitive boat), we want 0.5x Gain
        # If Actual is 0.5x Baseline (sluggish boat), we want 2x Gain
        target_kp = self.BASELINE_RESPONSIVENESS / measured_responsiveness
        
        # Dampen the change to avoid oscillation (Exponential Moving Average)
        # New gain = 95% old + 5% target (Very slow adaptation)
        new_kp = (self.pid.kp * 0.95) + (target_kp * 0.05)
        
        # Clamp Kp to sane limits to prevent runaway
        new_kp = max(0.2, min(5.0, new_kp))
        
        # Update PID
        self.pid.kp = new_kp
        
        # Scale I and D with P to maintain stability ratios
        # Ki is usually much smaller than Kp
        # Kd is usually roughly half of Kp (in some tuning methods) or smaller
        self.pid.ki = new_kp * 0.05 
        self.pid.kd = new_kp * 0.5
        
        print(f"AutoTune: Resp={measured_responsiveness:.4f}, TargetKp={target_kp:.2f} -> New PID: {new_kp:.2f}/{self.pid.ki:.3f}/{self.pid.kd:.2f}")
