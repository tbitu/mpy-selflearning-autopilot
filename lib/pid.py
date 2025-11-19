import utime as time

class PID:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._min_output, self._max_output = output_limits
        
        self._prev_error = 0
        self._integral = 0
        self._last_time = time.ticks_ms()
        
    def update(self, measurement):
        """Calculate PID output"""
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time) / 1000.0
        if dt <= 0: dt = 1e-3 # Prevent division by zero
        
        error = self.setpoint - measurement
        
        # Handle heading wrap-around (-180 to 180)
        if error > 180: error -= 360
        elif error < -180: error += 360
            
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self._integral += error * dt
        i_term = self.ki * self._integral
        
        # Derivative term
        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Clamp output
        if self._min_output is not None:
            output = max(self._min_output, output)
        if self._max_output is not None:
            output = min(self._max_output, output)
            
        # Anti-windup: clamp integral if output is clamped
        if self._max_output is not None and output == self._max_output and error > 0:
            self._integral -= error * dt # Back-calculate or just stop integrating
        elif self._min_output is not None and output == self._min_output and error < 0:
            self._integral -= error * dt
            
        self._prev_error = error
        self._last_time = now
        
        return output
        
    def reset(self):
        self._integral = 0
        self._prev_error = 0
        self._last_time = time.ticks_ms()
