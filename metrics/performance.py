import numpy as np

class PerformanceMetrics:
    """
    Calculating performance metrics of the response
    """
    @staticmethod
    def overshoot(reference, theta_array):    # calculates overshoot in percentage
        return round(max(theta_array - reference) * 100 / (reference[0]), 1)

    @staticmethod
    def rms(reference, theta_array):          # calculates rms value of the signal
        return np.sqrt(np.mean((theta_array - reference) ** 2))
    
    @staticmethod
    def settling(reference, theta_array, t, tol = 0.02):    # calculates settling time based on value of tol (currently set to 2%)
        index = np.where(~((np.abs(theta_array - reference)) <= (tol * reference)))[0]
        return t[index[-1] if index.size > 0 else 0]