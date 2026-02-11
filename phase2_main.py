# -------------------------------
# Importing modules
# -------------------------------
import numpy as np
import matplotlib.pyplot as plt

from models.actuator import ActuatorModel
from control.pid import PIDController
from simulation.simulator_phase2 import DualSimulator
from metrics.performance import PerformanceMetrics
from models.digital_twin import DigitalTwin
from diagnostics.residuals import ResidualGenerator

# -------------------------------
# Real model parameters
# -------------------------------
j_real = 0.0035        # inertia [kg-m^2]
b_real = 0.025         # damping [Nms/rad]
Kt_real= 0.05         # torque constant [Nm/A]

# -------------------------------
# Twin model parameters
# -------------------------------
j_twin = 0.0031
b_twin = 0.022
Kt_twin = 0.047

# -------------------------------
# Initializing values
# -------------------------------
dt = 0.001
duration = 2
t = np.arange(0, duration, dt)

reference = np.deg2rad(60) * np.ones_like(t)

# -------------------------------
# Controller gains
# -------------------------------
Kp = 4
Ki = 0.05
Kd = 0.2

# -------------------------------
# Calculating joint angle values
# -------------------------------
real_actuator = ActuatorModel(j_real, b_real, Kt_real)
twin_actuator = DigitalTwin(j_twin, b_twin, Kt_twin)
controller = PIDController(Kp, Ki, Kd)
simul = DualSimulator(real_actuator, twin_actuator, controller, dt)

real_theta, real_omega, twin_theta, twin_omega = simul.run(reference, t)

# -------------------------------
# Calculating residuals and printing outputs
# -------------------------------
theta_diff = ResidualGenerator.position(real_theta, twin_theta)
omega_diff = ResidualGenerator.velocity(real_omega, twin_omega)

settling_time = PerformanceMetrics.settling(reference, real_theta, t)
indices = np.where(t >= settling_time)
first_index = indices[0][0]
ss_mean = round(np.rad2deg(np.mean(theta_diff[first_index:])), 1)       # steady state residual mean

rms = round(np.rad2deg(np.sqrt(np.mean(theta_diff**2))), 1)
residual_mean = round(np.rad2deg(np.mean(theta_diff)), 1)
std_dev = round(np.rad2deg(theta_diff.std()), 1)

print("Steady state residual mean:             ", ss_mean, "deg")
print("Residual RMS:                           ", rms, "deg")
print("Mean residual:                          ", residual_mean, "deg")
print("Standard deviation:                     ", std_dev, "deg")

output_data = f"Steady state residual mean: {ss_mean} deg\nResidual RMS: {rms} deg\n" \
    f"Mean residual: {residual_mean} deg\nStandard deviation: {std_dev} deg"
file_name = "results/phase2_residual_characteristics.txt"
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
ax1.plot(t, np.rad2deg(real_theta), label = 'Real actuator', color = 'green')
ax1.plot(t, np.rad2deg(twin_theta), label = 'Digital twin', color = 'orange')
ax1.plot(t, np.rad2deg(reference), label = 'Setpoint', color = 'grey', linestyle = '--')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta (deg)")
ax1.set_title("Real vs Digital Twin")
ax1.legend()
plt.tight_layout()
fig1.savefig("results/phase2_real_vs_twin.png")

fig2, (ax1) = plt.subplots(1, 1, figsize = (8, 6))
ax1.plot(t, np.rad2deg(theta_diff), label = 'Residual theta', color = 'blue')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta residual (deg)")
ax1.set_title("Residual (Real - Twin)")
ax1.legend()
plt.tight_layout()
fig2.savefig("results/phase2_residual.png")
plt.show()