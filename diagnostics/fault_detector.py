import numpy as np

class FaultDetector:
    '''
    Detecting fault type using thresholds
    '''
    def __init__(self, pos_threshold, vel_threshold):
        self.pos_threshold = pos_threshold
        self.vel_threshold = vel_threshold

    def detect(self, pos_res, vel_res):
        pos_fault = np.abs(pos_res) > self.pos_threshold    # position fault when error in position is higher than the threshold
        vel_fault = np.abs(vel_res) > self.vel_threshold

        if pos_fault and vel_fault:
            return "Severe_fault"
        elif pos_fault:
            return "Position_fault"
        elif vel_fault:
            return "Velocity_fault"
        else:
            return "No_fault"