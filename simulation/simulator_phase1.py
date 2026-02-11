import numpy as np

class Simulator:
    """
    Closed-loop simulation
    """
    def __init__(self, actuator, controller, dt):
        self.actuator = actuator
        self.controller = controller
        self.dt = dt

    def run(self, reference, t):
        theta_array = np.zeros_like(t)    # joint angle
        omega_array = np.zeros_like(t)    # joint velocity

        self.actuator.reset()
        self.controller.reset()

        for i in range(1, len(t)):
            error = reference[i] - self.actuator.theta
            current = self.controller.compute(error, self.dt)

            theta, omega = self.actuator.step(current, self.dt)

            theta_array[i], omega_array[i] = theta, omega

        return theta_array, omega_array