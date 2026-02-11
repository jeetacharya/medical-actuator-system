import numpy as np

class DualSimulator:
    """
    Closed-loop simulation for real atuator and digital twin
    """
    def __init__(self, real, twin, controller, dt):
        self.real = real
        self.twin = twin
        self.controller = controller
        self.dt = dt

    def run(self, reference, t):
        theta_real = np.zeros_like(t)
        theta_twin = np.zeros_like(t)
        omega_real = np.zeros_like(t)
        omega_twin = np.zeros_like(t)

        self.real.reset()
        self.twin.reset()
        self.controller.reset()

        for i in range(1, len(t)):
            error = reference[i] - self.real.theta               # error and current are calculated from real model only
            current = self.controller.compute(error, self.dt)

            t_real, o_real = self.real.step(current, self.dt)
            t_twin, o_twin = self.twin.step(current, self.dt)    # same current is provided to digital twin model

            theta_real[i], omega_real[i] = t_real, o_real
            theta_twin[i], omega_twin[i] = t_twin, o_twin

        return theta_real, omega_real, theta_twin, omega_twin