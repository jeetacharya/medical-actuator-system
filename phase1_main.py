# -------------------------------
# Importing modules
# -------------------------------
import numpy as np
import matplotlib.pyplot as plt

from models.actuator import ActuatorModel
from control.pid import PIDController
from simulation.simulator_phase1 import Simulator
from metrics.performance import PerformanceMetrics

# -------------------------------
# Actuator model parameters
# -------------------------------
j = 0.0035        # inertia [kg-m^2]
b = 0.025         # damping [Nms/rad]
Kt = 0.05         # torque constant [Nm/A]

# -------------------------------
# Initializing values
# -------------------------------
dt = 0.001
duration = 2
t = np.arange(0, duration, dt)

reference = np.deg2rad(60) * np.ones_like(t)      # setpoint = 60 deg

# -------------------------------
# Controller gains
# -------------------------------
Kp = 4
Ki = 0.05
Kd = 0.2

# -------------------------------
# Calculating joint angle values
# -------------------------------
actuator = ActuatorModel(j, b, Kt)
controller = PIDController(Kp, Ki, Kd)
simul = Simulator(actuator, controller, dt)

theta, omega = simul.run(reference, t)

# -------------------------------
# Response characteristics
# -------------------------------
overshoot = PerformanceMetrics.overshoot(reference, theta)

rms_error = PerformanceMetrics.rms(reference, theta)
rms_error = round(rms_error * 180/np.pi, 1)

settling_time = PerformanceMetrics.settling(reference, theta, t)
settling_time = round(settling_time, 1)

# -------------------------------
# Printing outputs
# -------------------------------
print("Overshoot:                           ", overshoot, "%")
print("RMS error:                           ", rms_error, "deg")
print("Settling time:                       ", settling_time, "s")

output_data = f"Overshoot: {overshoot} % \nRMS error: {rms_error} deg \nSettling time: {settling_time} s"
file_name = "results/phase1_response_characteristics.txt"
try:
    with open(file_name, 'w') as f:
        f.write(output_data)
    print(f"Successfully saved output to {file_name}")
except IOError as e:
    print(f"Error saving file: {e}")

# -------------------------------
# Plotting graphs
# -------------------------------
fig1, (ax1) = plt.subplots(1, 1, figsize = (8, 6))
ax1.plot(t, np.rad2deg(theta), label = 'Actual joint angle', color = 'blue')
ax1.plot(t, np.rad2deg(reference), label = 'Setpoint', color = 'grey', linestyle = '--')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta (deg)")
ax1.set_title("Joint angle vs Time")
ax1.legend()
fig1.savefig("results/phase1_position_tracking.png")
plt.show()