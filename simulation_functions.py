import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as op
from scipy.integrate import odeint

#%% Vehicle parameters
# Define fitting sub-functions
def quadratic_fit(x, a, b, c):
    return a * x**2 + b * x + c

def constant_fit(x, d):
    return d

def linear_fit(x, e, f):
    return e * x + f

# Define the full piecewise model (vectorized)
def make_piecewise_model_vectorized(transition_point1, transition_point2):
    def model(x, a, b, c, d, e, f):
        return np.piecewise(
            x,
            [x < transition_point1,
             (x >= transition_point1) & (x < transition_point2),
             x >= transition_point2],
            [lambda x: quadratic_fit(x, a, b, c),
             lambda x: constant_fit(x, d),
             lambda x: linear_fit(x, e, f)]
        )
    return model

def fit_motor_curve(motor_data, transition_point1=1000, transition_point2=5000):
    RPM = motor_data['RPM'].values
    Torque = motor_data['Torque'].values
    model = make_piecewise_model_vectorized(transition_point1, transition_point2)
    initial_guess = (-1, 1, 1, 1, -1, 1)
    popt, _ = op.curve_fit(model, RPM, Torque, initial_guess)
    return popt, model

def plot_motor_fit(motor_data, fit_params, model):
    RPM = motor_data['RPM'].values
    Torque = motor_data['Torque'].values
    fit_curve = model(RPM, *fit_params)

    plt.plot(RPM, Torque, 'o', label='Data')
    plt.plot(RPM, fit_curve, '-', label='Piecewise Fit')
    plt.xlabel('RPM')
    plt.ylabel('Torque')
    plt.legend()
    plt.title('Motor Torque vs RPM')
    plt.grid(True)
    plt.show()


def fit_cornering_stiffness(latg, force_data):
    initial_guess=(0.1, 1, 0)
    popt, _ = op.curve_fit(quadratic_fit, latg, force_data, initial_guess)
    return popt

def plot_cornering_stiffness(latg, front_data, rear_data, front_params, rear_params):
    front_fit = quadratic_fit(latg, *front_params)
    rear_fit = quadratic_fit(latg, *rear_params)

    front_eq = f'Front: y={front_params[0]:.2f}x² + {front_params[1]:.2f}x + {front_params[2]:.2f}'
    rear_eq = f'Rear:  y={rear_params[0]:.2f}x² + {rear_params[1]:.2f}x + {rear_params[2]:.2f}'

    plt.plot(latg, front_data, 'b.', label='Front Data')
    plt.plot(latg, front_fit, 'r-', label='Front Fit')

    plt.plot(latg, rear_data, 'g.', label='Rear Data')
    plt.plot(latg, rear_fit, 'orange', label='Rear Fit')

    plt.xlabel('Lateral Acceleration (g)')
    plt.ylabel('Cornering Force')
    plt.title('Cornering Stiffness Calibration')
    plt.legend()
    plt.text(0.05, 0.1, front_eq, transform=plt.gca().transAxes)
    plt.text(0.05, 0.03, rear_eq, transform=plt.gca().transAxes)
    plt.grid(True)
    plt.savefig('cornering_stiffness_vs_latg.png')
    plt.show()

#%% Track Builder
def track_builder(track):
  # --- Initialize track ---
  ds = .01  # distance step. How precise the length/radii can be
  x_list = [0.0]
  y_list = [0.0]
  heading = 0.0

  for idx, row in track.iterrows():
      ttype = row['Type'].strip().lower()
      length = float(row['Length'])
      radius = row['Radius'] if pd.notna(row['Radius']) else None
      n_points = int(length / ds)

      if ttype == 'straight':
          # Straight segment
          for _ in range(n_points):
              x_list.append(x_list[-1] + ds * np.cos(heading))
              y_list.append(y_list[-1] + ds * np.sin(heading))

      else:
          # Corner segment
          curvature = 1 / float(radius)
          dtheta = ds * curvature if ttype == 'left' else -ds * curvature
          for _ in range(n_points):
              heading += dtheta
              x_list.append(x_list[-1] + ds * np.cos(heading))
              y_list.append(y_list[-1] + ds * np.sin(heading))

  # Convert to arrays
  x = np.array(x_list)
  y = np.array(y_list)
  #x[-1]=0
  #y[-1]=0

  #Calculate cumulative distance
  s = np.zeros_like(x)
  if len(x)>1:
    s[1:] = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))

  if len(s)>1:
    dx = np.gradient(x, s)
    dy = np.gradient(y, s)
    ddx = np.gradient(dx, s)
    ddy = np.gradient(dy, s)


  # Compute curvature (x'*y''-y'*x'')/(x'^2+y'^2)^1.5
  curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5

  # --- Track Visualization ---
  plt.figure(figsize=(8,4))
  plt.plot(x, y, label='Track Centerline', color='blue')
  plt.title('Track Layout')
  plt.xlabel('X (m)')
  plt.ylabel('Y (m)')
  plt.legend()
  plt.show()
  return[s,ds,curvature,x,y]

#%% Simulation functions

def derivatives(state,t,Cf_inst,Cr_inst,delta,a,b,Iz,m,ax,F_drag):
    vx, vy, r = state
    
    if vx < 0.1:
        vx = 0.1

    alpha_f = delta - (vy + a * r) / vx
    alpha_r = - (vy - b * r) / vx

    Fyf = Cf_inst * alpha_f
    Fyr = Cr_inst * alpha_r

    dvx = ax - r * vy - F_drag / m
    dvy = (Fyf + Fyr) / m + r * vx
    dr = (a * Fyf - b * Fyr) / Iz
    return [dvx, dvy, dr]


def run_simulation(state, states, velocities, time, s, ds, curvature, desired_delta, popt, m, rho, A, Cd, Cl, g, a, b, Iz, mu,
                    ay_max_limit, ax_brake, gear_ratio, tire_radius,front_params,rear_params,model):
    """
    Runs the vehicle simulation along the track.

    Args:
        state: Initial state of the vehicle [vx, vy, r].
        states: List to store the state history.
        velocities: List to store the velocity history.
        time: List to store the time history.
        s: Array of cumulative track distances. Generated from the Track builder
        ds: distance step from Track Builder
        curvature: Array of track curvature values. Generated from the Track Builder
        desired_delta: Array of desired steering angles.
        popt: Parameters for the piecewise motor torque model.
        m: Vehicle mass.
        rho: Air density.
        A: Frontal area.
        Cd: Drag coefficient.
        Cl: Lift coefficient.
        g: Gravity.
        a: Distance from CG to front axle.
        b: Distance from CG to rear axle.
        Iz: Yaw moment of inertia.
        mu: Friction coefficient.
        ay_max_limit: Maximum allowed lateral acceleration.
        ax_brake: Maximum braking deceleration.
        gear_ratio: Gear ratio.
        tire_radius: Tire radius.


    Returns:
        Tuple containing:
            states: List of vehicle states over time.
            velocities: List of vehicle velocities over time.
            time: List of simulation time points.
            laptime: Estimated laptime.
    """


    for i in range(1, len(s)):
        delta = desired_delta[i]

        vx = state[0]
        vy=state[1]
        #r=state[2]

        v_total=np.sqrt(vx**2 + vy**2)
        F_drag = 0.5 * rho * A * Cd * v_total**2
        F_down = -0.5 * rho * A * Cl * v_total**2

        # Total Normal force
        Fz= (m* g + F_down)/2

        # Max lateral acceleration with downforce allowed by tires
        ay_tire_limit = mu * Fz / m

        ay_desired_track = vx**2 * curvature[i] if curvature[i] != 0 else 0

        ay_desired=np.sign(ay_desired_track)*min(abs(ay_desired_track), ay_tire_limit,ay_max_limit)

        ax_available = np.sqrt(max((ay_tire_limit)**2 - ay_desired**2, 0))

        current_rpm=(vx/tire_radius)*gear_ratio*(60/(2*np.pi))

        available_torque=model(current_rpm,*popt)

        F_engine=available_torque*gear_ratio/tire_radius

        ax_available_engine=F_engine/m


        # Determine acceleration / braking
        if vx < 30:
            ax = min(ax_available_engine, ax_available)
        else:
            ax = max(ax_brake, -ax_available)

        dt = ds / max(vx, 0.1)  # Avoid zero division
        t_span=[0,dt]

        Cf_inst=quadratic_fit(ay_desired,front_params[0],front_params[1],front_params[2])
        Cr_inst=quadratic_fit(ay_desired,rear_params[0],rear_params[1],rear_params[2])

        state = odeint(derivatives, state, t_span, args=(Cf_inst, Cr_inst, delta, a, b, Iz, m, ax, F_drag))[-1]


        states.append(state)
        velocities.append(state[0])
        time.append(time[-1] + dt)

    laptime = time[-1]
    return states, velocities, time, laptime

def plot_simulation_results(s, velocities, x, y, time):
    """
    Plots the velocity profile along the track and the track layout
    with velocity colormap.

    Args:
        s: Array of cumulative track distances.
        velocities: List or array of vehicle velocities.
        x: Array of track x-coordinates.
        y: Array of track y-coordinates.
        time: List or array of simulation time points.
    """
    fig, axs = plt.subplots(1, 2, figsize=(14, 5))

    # --- Subplot 1: Velocity Profile ---
    axs[0].plot(s, velocities, color='green')
    axs[0].set_title('Velocity Profile along the Track')
    axs[0].set_xlabel('Track Position (m)')
    axs[0].set_ylabel('Speed (m/s)')
    axs[0].grid(True)

    # --- Subplot 2: Track Layout with Velocity Colormap ---
    norm = plt.Normalize(min(velocities), max(velocities))
    cmap = plt.cm.viridis
    sc = axs[1].scatter(x, y, c=velocities, cmap=cmap, norm=norm, s=5)
    axs[1].set_title('Track Layout with Velocity Profile')
    axs[1].set_xlabel('X (m)')
    axs[1].set_ylabel('Y (m)')

    # Add colorbar
    cbar = fig.colorbar(sc, ax=axs[1], label='Speed (m/s)')

    plt.tight_layout()
    plt.show()