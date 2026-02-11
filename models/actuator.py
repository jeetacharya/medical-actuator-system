import numpy as np

class ActuatorModel:
    """
    Rotational actuator model
    """
    def __init__(self, J, b, Kt, tau_load = 0.0):
        self.J = J                   # actuator parameters
        self.b = b
        self.Kt = Kt
        self.tau_load = tau_load

        self.theta = 0.0             # initializing values
        self.omega = 0.0

    def reset(self):
        self.theta = 0.0
        self.omega = 0.0

    def step(self, current, dt):
        domega = ((self.Kt * current) - (self.b * self.omega) - (self.tau_load)) / (self.J)
        dtheta = self.omega

        self.theta += dtheta * dt
        self.omega += domega * dt

        return self.theta, self.omega