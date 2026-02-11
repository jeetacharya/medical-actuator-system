import numpy as np

class Requirements:
    '''
    Defining thresholds for signal response characteristics
    '''
    MAX_OVERSHOOT = 10.0
    MAX_SETTLING_TIME = 1.0
    MAX_RMS_ERROR = np.deg2rad(17.15)
    MAX_FAULT_LATENCY = 0.2