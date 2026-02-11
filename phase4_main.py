# -------------------------------
# Importing modules
# -------------------------------
import numpy as np
import matplotlib.pyplot as plt
import csv

from models.actuator import ActuatorModel
from control.pid import PIDController
from simulation.simulator_phase4 import SafeSimulator
from models.digital_twin import DigitalTwin
from diagnostics.fault_detector import FaultDetector
from diagnostics.fault_injection import FaultInjector
from safety.state_machine import SafetyStateMachine
from risk.fmea import FMEA
from verification.requirements import Requirements
from verification.test_runner import TestRunner

# -------------------------------
# Initializing values
# -------------------------------
dt = 0.001
duration = 2
t = np.arange(0, duration, dt)

reference = np.deg2rad(60) * np.ones_like(t)

# -------------------------------
# Calculating joint angle values and printing outputs
# -------------------------------
real_actuator = ActuatorModel(0.0035, 0.025, 0.05)
twin_actuator = DigitalTwin(0.0033, 0.022, 0.047)
controller = PIDController(4, 0.05, 0.2)
fault_injector = FaultInjector(drift_rate = np.deg2rad(1))
fault_detector = FaultDetector(np.deg2rad(2), np.deg2rad(5))
safety_state = SafetyStateMachine()
requirements = Requirements()
simulator = SafeSimulator(real_actuator, twin_actuator, controller, fault_injector, fault_detector, safety_state, dt)

runner = TestRunner(requirements, simulator, dt)
results = runner.test_run(reference, t)
verification = runner.verify(results)

print("")
print("Verification Results")
for k, v in verification.items():
    print(f"{k}: {'Pass' if v else 'Fail'}")

verification_report = "results/phase4_verification_report.csv"
with open(verification_report, 'w', newline='') as f:
    fieldnames = verification.keys()
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerow(verification)

fmea_documentation = FMEA()
fmea_documentation.add_records("Position sensor", "Drift", 8, 4, 3)
fmea_documentation.add_records("Actuator", "Torque loss", 7, 2, 3)
fmea_documentation.add_records("Controller", "Integral windup", 9, 2, 4)

print("")
print("FMEA Report")
for i in fmea_documentation.fmea_report():
    print(i)

report_fmea = "results/phase4_fmea.csv"
with open(report_fmea, 'w', newline='') as f:
    fieldnames = fmea_documentation.fmea_report()[0].keys()
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(fmea_documentation.fmea_report())