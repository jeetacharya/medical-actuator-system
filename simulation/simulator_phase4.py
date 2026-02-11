import numpy as np

class SafeSimulator:
    """
    Closed-loop simulation with safety states incorporated
    """
    def __init__(self, real, twin, controller, injector, detector, safety, dt):
        self.real = real
        self.twin = twin
        self.controller = controller
        self.injector = injector
        self.detector = detector
        self.safety = safety
        self.dt = dt

    def run(self, reference, t):
        theta_real = np.zeros_like(t)
        theta_twin = np.zeros_like(t)
        states = []

        self.real.reset()
        self.twin.reset()
        self.controller.reset()

        for i in range(1, len(t)):
            error = reference[i] - self.real.theta
            current = self.controller.compute(error, self.dt)

            if self.safety.state == "Degraded_state":
                current *= 0.3
            elif self.safety.state == "Shutdown_state":
                current = 0.0

            t_real, o_real = self.real.step(current, self.dt)
            t_twin, o_twin = self.twin.step(current, self.dt)

            t_noisy = self.injector.apply(t_real, self.dt)

            pos_res = t_noisy - t_twin
            vel_res = o_real - o_twin

            fault_state = self.detector.detect(pos_res, vel_res)
            safety_state = self.safety.check(fault_state)

            theta_real[i] = t_noisy
            theta_twin[i] = t_twin
            states.append(safety_state)

        return theta_real, theta_twin, states