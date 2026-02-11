import numpy as np

class SafetyStateMachine:
    '''
    Classification of the state of the device
    '''
    def __init__(self):
        self.state = "Normal_state"

    def check(self, fault_status):
        if self.state == "Normal_state":
            if fault_status == "Position_fault" or fault_status == "Velocity_fault":
                self.state = "Degraded_state"
            elif fault_status == "Severe_fault":
                self.state = "Shutdown_state"

        elif self.state == "Degraded_state":
            if fault_status == "Severe_fault":
                self.state = "Shutdown_state"
            
        return self.state