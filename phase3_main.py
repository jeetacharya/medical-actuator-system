# -------------------------------
# Importing modules
# -------------------------------
import numpy as np
import matplotlib.pyplot as plt

from models.actuator import ActuatorModel
from control.pid import PIDController
from simulation.simulator_phase3 import SafeSimulator
from models.digital_twin import DigitalTwin
from diagnostics.fault_detector import FaultDetector
from diagnostics.fault_injection import FaultInjector
from safety.state_machine import SafetyStateMachine

# -------------------------------
# Initializing values
# -------------------------------
dt = 0.001
duration = 2
t = np.arange(0, duration, dt)

reference = np.deg2rad(60) * np.ones_like(t)

# -------------------------------
# Calculating values and printing outputs
# -------------------------------
real_actuator = ActuatorModel(0.0035, 0.025, 0.05)
twin_actuator = DigitalTwin(0.0033, 0.022, 0.047)
controller = PIDController(4, 0.05, 0.2)
fault_injector = FaultInjector(drift_rate = np.deg2rad(1))
fault_detector = FaultDetector(np.deg2rad(2), np.deg2rad(5))
safety_state = SafetyStateMachine()

simul = SafeSimulator(real_actuator, twin_actuator, controller, fault_injector, fault_detector, safety_state, dt)

theta_real, theta_twin, states, current, fault_type = simul.run(reference, t)

index_position = fault_type.index("Position_fault")
t_position = round(t[index_position], 2)
index_velocity = fault_type.index("Velocity_fault")
t_velocity = round(t[index_velocity], 2)
detection_latency = min(t_position, t_velocity)
index_degraded = states.index("Degraded_state")
t_degraded = round(t[index_degraded], 2)
index_shutdown = states.index("Shutdown_state")
t_shutdown = round(t[index_shutdown], 2)
duration_degraded = t_shutdown - t_degraded            # total time spent in degraded state

print("Position fault occurs at:          ", t_position, "sec")
print("Velocity fault occurs at:          ", t_velocity, "sec")
print("Detection latency:                 ", detection_latency, "sec")
print("Degraded state begins at:          ", t_degraded, "sec")
print("Shutdown state begins at:          ", t_shutdown, "sec")
print("Time spent in degraded mode:       ", duration_degraded, "sec")
print("Fault growth rate:                 ", "1 deg/s")
print("Time to detection:                 ", detection_latency, "sec")

output_data = f"Position fault occurs at: {t_position} sec\nVelocity fault occurs at: {t_velocity} sec\n" \
    f"Detection latency: {detection_latency} sec\nDegraded state begins at: {t_degraded} sec\n" \
    f"Shutdown state begins at: {t_shutdown} sec\nTime spent in degraded mode: {duration_degraded} sec\n" \
    f"Fault growth rate: 1 deg/sec\nTime to detection: {detection_latency} sec"
file_name = "results/phase3_fault_state.txt"
try:
    with open(file_name, 'w') as f:
        f.write(output_data)
    print(f"Successfully saved output to {file_name}")
except IOError as e:
    print(f"Error saving file: {e}")

# -------------------------------
# Plotting graphs
# -------------------------------
fig1, (ax1) = plt.subplots(1, 1, figsize = (8,6))
ax1.plot(t, np.rad2deg(theta_real), label = 'Faulty signal', color = 'orange')
ax1.plot(t, np.rad2deg(theta_twin), label = 'Digital twin', color = 'green')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta (deg)")
ax1.set_title("Fault Detection and Safety Response")
ax1.legend()
plt.tight_layout()
fig1.savefig("results/phase3_fault_injection.png")

fig2, (ax1) = plt.subplots(1, 1, figsize = (8,6))
ax1.plot(t, np.rad2deg(theta_real), label = 'Faulty signal', color = 'orange')
ax1.plot(t, np.rad2deg(theta_twin), label = 'Digital twin', color = 'green')
ax1.axvline(x = t[index_degraded], color = 'grey', linestyle = '--')
ax1.axvline(x = t[index_shutdown], color = 'grey', linestyle = '--')
ax1.text(-0.1, 10, "Normal", fontsize = 12, color = 'blue')
ax1.text(t[index_degraded] + 0.01, 0.5, "Degraded", fontsize = 12, color = 'blue')
ax1.text(t[index_shutdown] + 0.7, 0.5, "Shutdown", fontsize = 12, color = 'blue')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta (deg)")
ax1.set_title("Safety State Classification")
ax1.legend()
plt.tight_layout()
fig2.savefig("results/phase3_safety_state.png")

fig3, (ax1) = plt.subplots(1, 1, figsize = (8,6))
ax1.plot(t, current, label = 'Current', color = 'orange')
ax1.axvline(x = t[index_degraded], color = 'grey', linestyle = '--')
ax1.text(t[index_degraded] + 0.01, 2, "Degraded current", fontsize = 12, color = 'blue')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Current (A)")
ax1.set_title("Current vs Time")
ax1.legend()
plt.tight_layout()
fig3.savefig("results/phase3_control_limiting.png")
plt.show()