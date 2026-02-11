import numpy as np

class FMEA:
    '''
    FMEA documentation
    '''
    def __init__(self):
        self.records = []

    def add_records(self, component, failure, severity, occurrence, detection):
        rpn = severity * occurrence * detection
        self.records.append({                         # adding records depending on failure modes
            "Component": component,
            "Failure mode": failure,
            "Severity": severity,
            "Occurrence": occurrence,
            "Detection": detection,
            "RPN": rpn
        })

    def fmea_report(self):                           # report generation
        return sorted(self.records, key = lambda x: x["RPN"], reverse = True)