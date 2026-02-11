import numpy as np
from metrics.performance import PerformanceMetrics

class TestRunner:
    '''
    Running a test and verifying the final report against thresholds
    '''
    def __init__(self, requirements, simulator, dt):
        self.requirements = requirements
        self.simulator = simulator
        self.dt = dt

    def test_run(self, reference, t):
        theta_real, theta_twin, states = self.simulator.run(reference, t)

        overshoot = PerformanceMetrics.overshoot(reference, theta_real)
        rms_error = PerformanceMetrics.rms(reference, theta_real)
        settling_time = PerformanceMetrics.settling(reference, theta_real, t)

        latency_index = next((i for i, s in enumerate(states) if s != "Normal_state"), None)
        fault_latency = latency_index * self.dt if latency_index else np.nan         # calculating fault latency

        return {"overshoot": overshoot, "rms_error": rms_error, "settling_time": settling_time, "fault_latency": fault_latency}
    
    def verify(self, results):            # verifying that the results obtained are lower than their thresholds
        return {
            "Overshoot": results["overshoot"] <= self.requirements.MAX_OVERSHOOT,
            "Rms_error": results["rms_error"] <= self.requirements.MAX_RMS_ERROR,
            "Settling_time": results["settling_time"] <= self.requirements.MAX_SETTLING_TIME,
            "Fault_latency": results["fault_latency"] <= self.requirements.MAX_FAULT_LATENCY
        }