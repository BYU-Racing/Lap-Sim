import numpy as np
import pandas as pd
from simulation_functions import *

# --- Vehicle Parameters ---
m = 200         # Mass (kg)
Iz = 85.2        # Yaw moment of inertia (kg*m^2)
a = .816          # CG to front axle (m)
b = .823          # CG to rear axle (m)
mu = 1.6         # Friction coefficient
g = 9.81         # Gravity (m/s^2)
gear_ratio=4.14
tire_radius=.203 #meters

ax_brake = -1.4*g  # Max braking deceleration (m/s^2)
ay_max_limit=1.5*g #max lateral acceleration (m/s^2)
C_rr=.015 #rolling resistance coefficient

# --- Cornering Stiffness ---
data = np.array(pd.read_csv('cornering_stiffness_vs_latg.csv'))
latg_data = data[:, 0]
Front_data = data[:, 1]
Rear_data = data[:, 2]

# Fit front and rear cornering stiffness curves
front_params = fit_cornering_stiffness(latg_data, Front_data)
rear_params = fit_cornering_stiffness(latg_data, Rear_data)


# --- Aerodynamic parameters ---
rho = 1.225      # Air density (kg/m^3)
A = 2.2          # Frontal area (m^2)
Cd = 0.6
Cl = -1      # Negative lift = downforce


#--- motor data---
motor = pd.read_csv('208 motor data.csv')
params, model = fit_motor_curve(motor) #find fit for motor data

#--- Track---
track=pd.read_csv('2024 Endurance.csv') #or whatever the name of the track file is
s,ds,curvature,x,y=track_builder(track)

#---Setup---
state = np.array([0.0, 0.0, 0.0])  # Initial state [vx, vy, r]
states = [state]
velocities = [state[0]]
time = [0]
desired_delta = np.zeros_like(s)

#---Run Simulation---
states, velocities, time, laptime = run_simulation(
    state, states, velocities, time,
    s, ds, curvature, desired_delta, params,
    m, rho, A, Cd, Cl, g, a, b, Iz, mu,
    ay_max_limit, ax_brake, gear_ratio, tire_radius,
    front_params, rear_params,model,C_rr
)

#---Laptime---
print(f"Estimated Laptime: {laptime:.2f} seconds")

#---Plotting Results---
plot_simulation_results(s,velocities,x,y,time)
#%% debugging
# Plot the motor fit.
#plot_motor_fit(motor, params, model)

#plot the cornering stiffness fit
#plot_cornering_stiffness(latg_data, Front_data, Rear_data, front_params, rear_params)
