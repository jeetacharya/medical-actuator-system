import numpy as np

class PIDController:
    """
    PID Controller for Model
    """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp    # controller gains
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0    # initializing values
        self.error_int = 0.0

    def reset(self):
        self.prev_error = 0.0    # resetting values
        self.error_int = 0.0

    def compute(self, error, dt):
        self.error_int += error * dt
        error_diff = (error - self.prev_error) / dt

        current = (self.Kp * error) + (self.Ki * self.error_int) + (self.Kd * error_diff)    # calculating input current to system

        self.prev_error = error

        return current