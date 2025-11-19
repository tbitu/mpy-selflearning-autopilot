import machine
import utime as time

class MotorController:
    def __init__(self, pwm_pin, dir_pin, freq=1000):
        self.pwm = machine.PWM(machine.Pin(pwm_pin), freq=freq, duty=0)
        self.dir_pin = machine.Pin(dir_pin, machine.Pin.OUT)
        self.active = False
        self.start_time = 0
        self.duration_ms = 0
        self.direction = "stop"

    def run(self, direction, duration_ms, power=1023):
        """Start the motor in a direction for a duration (non-blocking)"""
        if direction not in ["port", "starboard"]:
            self.stop()
            return

        self.direction = direction
        self.duration_ms = duration_ms
        self.start_time = time.ticks_ms()
        self.active = True
        
        # Set direction
        self.dir_pin.value(1 if direction == "starboard" else 0)
        # Set power
        self.pwm.duty(power)

    def stop(self):
        """Stop the motor immediately"""
        self.pwm.duty(0)
        self.active = False
        self.direction = "stop"

    def update(self):
        """Check if motor should stop (call this in main loop)"""
        if not self.active:
            return

        if time.ticks_diff(time.ticks_ms(), self.start_time) >= self.duration_ms:
            self.stop()
            
    def is_running(self):
        return self.active
