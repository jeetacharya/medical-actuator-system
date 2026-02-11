import numpy as np

class FaultInjector:
    '''
    Introducing drift into signal
    '''
    def __init__(self, drift_rate = 0.0):
        self.drift_rate = drift_rate    # (deg/sec)
        self.bias = 0.0

    def apply(self, value, dt):
        self.bias += self.drift_rate  * dt    # self.bias accumulates over time
        return value + self.bias