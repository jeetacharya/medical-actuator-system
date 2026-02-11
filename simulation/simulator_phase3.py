import numpy as np

class SafeSimulator:
    """
    Closed-loop simulation with safety states incorporated
    """
    def __init__(self, real, twin, controller, injector, detector, safety, dt):
        self.real = real
        self.twin = twin
        self.controller = controller
        self.injector = injector            # fault injector
        self.detector = detector            # fault detector
        self.safety = safety
        self.dt = dt

    def run(self, reference, t):
        theta_real = np.zeros_like(t)
        theta_twin = np.zeros_like(t)
        states = []                         # list of safety states
        current_adjusted = [0]
        fault_type = ["No_fault"]

        self.real.reset()
        self.twin.reset()
        self.controller.reset()

        for i in range(1, len(t)):
            error = reference[i] - self.real.theta
            current = self.controller.compute(error, self.dt)

            if self.safety.state == "Degraded_state":             # current is reduced when its in degraded state
                current *= 0.3
            elif self.safety.state == "Shutdown_state":           # current is not provided when its in shutdown state
                current = 0.0

            t_real, o_real = self.real.step(current, self.dt)
            t_twin, o_twin = self.twin.step(current, self.dt)

            t_noisy = self.injector.apply(t_real, self.dt)        # drift fault is applied to the real actuator's joint angle

            pos_res = t_noisy - t_twin
            vel_res = o_real - o_twin

            fault_state = self.detector.detect(pos_res, vel_res)    # detecting faults
            safety_state = self.safety.check(fault_state)           # checking the safety state

            theta_real[i] = t_noisy
            theta_twin[i] = t_twin
            states.append(safety_state)
            current_adjusted.append(current)
            fault_type.append(fault_state)

        return theta_real, theta_twin, states, current_adjusted, fault_type