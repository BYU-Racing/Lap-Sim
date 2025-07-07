import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as op
from scipy.integrate import odeint
import math

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


def fit_cornering_stiffness(latg,Fy_data,initial_guess=(.1,1,0)):
    popt, _ = op.curve_fit(quadratic_fit,latg, Fy_data, initial_guess)
    return popt *2.2 #relates testing method to track surface

def plot_cornering_stiffness(datasets,labels,fit_params,title='Cornering Stiffness fit'):
    plt.figure(figsize=(10,6))
    for (latg,Fy),label,params in zip(datasets,labels,fit_params):
        force_fit=quadratic_fit(latg,*params)
        plt.plot(latg,Fy, 'o', label=f'{label} Data')
        plt.plot(latg,force_fit,'-', label=f'{label} Fit')

    plt.xlabel('Lateral Acceleration (g)')
    plt.ylabel('Cornering Force')
    plt.title(title)
    plt.legend()
    #plt.text(0.05, 0.1, eq, transform=plt.gca().transAxes)
    plt.grid(True)
    plt.show()

#%% Track Builder
def detect_track_type(csv_path):
   filename=os.path.basename(csv_path).lower()

   keyword_map={
      "accel":["Acceleration","accel"],
      "skid": ["Skidpad", "skid"],
      "auto": ["Autocross", "auto"],
      "endur": ["Endurance", "end","endure" ]
   }
   for key,keywords in keyword_map.items():
      for word in keywords:
         if word in filename:
            return key
   return None

def track_builder(track,sector_boundaries):
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
  #to make it a closed circuit uncomment these
  #x[-1]=0
  #y[-1]=0

  #Calculate cumulative distance
  s = np.zeros_like(x)
  if len(x)>1:
    s[1:] = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))

  sector_list=np.zeros_like(s,dtype=int)
  for i in range(1,len(sector_boundaries)):
      in_sector = (s>=sector_boundaries[i-1]) & (s<sector_boundaries[i])
      sector_list[in_sector]=i
  sector_list[s >= sector_boundaries[-1]]=len(sector_boundaries)-1

  if len(s)>1:
    dx = np.gradient(x, s)
    dy = np.gradient(y, s)
    ddx = np.gradient(dx, s)
    ddy = np.gradient(dy, s)


  # Compute curvature (x'*y''-y'*x'')/(x'^2+y'^2)^1.5
  curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5

  return[s,ds,curvature,x,y,sector_list]

#%% Simulation functions

def car_model_derivatives(state, t, Cf_inst, Cr_inst, steering_angle, a, b, Iz, m, ax, F_drag, track_width):
    vx, vy, r = state
    vx=np.clip(vx,0.1,200)

    vy_f=vy+a*r
    vy_r=vy-b*r
    v_track=(track_width/2)*r

    if 0< math.degrees(steering_angle):
        alpha_f_left = steering_angle - (vy_f + v_track) / vx
        alpha_f_right = steering_angle - (vy_f -v_track) / vx
        alpha_r_left = - (vy_r + v_track) / vx
        alpha_r_right = - (vy_r - v_track) / vx

    elif  math.degrees(steering_angle)<0:
        alpha_f_left= steering_angle - (vy_f - v_track) / vx
        alpha_f_right= steering_angle- (vy_f + v_track) / vx
        alpha_r_left= - (vy_r - v_track) / vx
        alpha_r_right= - (vy_r + v_track) / vx

    elif 0==math.degrees(steering_angle):
        alpha_f_left=alpha_f_right=alpha_r_left=alpha_r_right=0

    # Tire forces (assuming symmetrical behavior)
    Fyf_left = Cf_inst * alpha_f_left
    Fyf_right = Cf_inst * alpha_f_right
    Fyr_left = Cr_inst * alpha_r_left
    Fyr_right = Cr_inst * alpha_r_right

    Fyf = (Fyf_left + Fyf_right)
    Fyr = (Fyr_left + Fyr_right)

    # Dynamics
    dvx = ax - r * vy - F_drag / m
    dvy = (Fyf + Fyr) / m + r * vx
    dr = (a * (Fyf_right + Fyf_left) - b * (Fyr_right + Fyr_left)+(track_width/2)*(Fyf_right-Fyf_left+Fyr_right-Fyr_left)) / Iz
    return [dvx, dvy, dr]


# Update the simulation loop to use car model
def simulate_lap(state, states, velocities, time, track, vehicle, human_factor,ax_actual=0.0):
    s=track.s
    curvature=track.curvature
    ds=track.ds
    sector_list=track.sector_list
    sector_times={}
    current_sector=sector_list[0]
    sector_start_time=time[0]
    v_max_profile=track.v_max_profile
    m=vehicle.general.m
    mu=vehicle.general.mu
    g=vehicle.general.g
    rho=vehicle.general.rho
    A=vehicle.aero.A
    Cd=vehicle.aero.Cd
    Cl=vehicle.aero.Cl
    C_rr=vehicle.general.C_rr
    a=vehicle.suspension.a
    b=vehicle.suspension.b
    l=vehicle.suspension.l
    h=vehicle.suspension.h
    tire_radius=vehicle.general.tire_radius
    gear_ratio=vehicle.motor.gear_ratio
    model=vehicle.motor.model
    popt=vehicle.motor.params
    ax_brake=vehicle.general.ax_brake
    ay_max_limit=vehicle.general.ay_max_limit
    front_params=vehicle.suspension.front_params
    rear_params=vehicle.suspension.rear_params
    Iz=vehicle.suspension.Iz
    human_factor=human_factor
    track_wdith=vehicle.suspension.track_width

    vx_safe=.1 
    W=m*g
    response_time=.2 #delay between acceleration / braking input and actual acceleration / braking
    time_s=[0.0]



    for i in range(1, len(s)):
        human=np.random.uniform(human_factor,1)
        vx = state[0]
        vy = state[1]
        v_total = np.sqrt(vx**2 + vy**2)

        F_drag = 0.5 * rho * A * Cd * v_total**2
        F_down = -0.5 * rho * A * Cl * v_total**2
        F_rr = C_rr * W

        Fz = (W + F_down) / 2
        tire_limit = mu * Fz / m *human
        ay_desired_track = vx_safe**2 * curvature[i] if curvature[i] != 0 else 0
        ay_desired = np.sign(ay_desired_track) * min(abs(ay_desired_track), tire_limit, ay_max_limit)

        ax_available = np.sqrt(tire_limit**2 - ay_desired**2)

        current_rpm = (vx / tire_radius) * gear_ratio * (60 / (2 * np.pi))
        available_torque = model(current_rpm, *popt)
        F_engine = available_torque * gear_ratio / tire_radius
        F_total = F_engine - F_drag - F_rr
        ax_available_engine = F_total / m
        
        lookahead_dist = 7 #how far ahead it will consider braking (m)
        s_current = s[i]
        current_sector=sector_list[i]
        
        within_range=np.where(s<s_current+lookahead_dist)[0]
        lookahead_idx=within_range[-1] if len(within_range) >0 else i
        lookahead_idx= min(lookahead_idx,len(s)-1)

        v_lookahead_target=np.min(v_max_profile[i:lookahead_idx+1])

        if vx > v_lookahead_target:
            ax_cmd = min(-ax_brake, -ax_available)
        else:
            ax_cmd = np.clip(ax_available_engine, -ax_brake, ax_available)

        dt =max( ds / max(vx, 0.1), 1e-4)
        t_start=time[-1]
        t_end=t_start+dt
        t_span = [t_start, t_end]
        
        ax_actual+=((ax_cmd-ax_actual)*dt/response_time)*human

        if current_sector not in sector_times:
            sector_times[current_sector]=0.0
        sector_times[current_sector]+=dt

        Cf_inst = quadratic_fit(ay_desired, front_params[0], front_params[1], front_params[2])
        Cr_inst = quadratic_fit(ay_desired, rear_params[0], rear_params[1], rear_params[2])

        steering_angle = l*ay_desired/ max(vx,.1)**2 #still bicycle model but the derivatives fixes it
        state = odeint(car_model_derivatives, state, t_span, args=(Cf_inst, Cr_inst, steering_angle, a, b, Iz, m, ax_actual, F_drag,track_wdith))[-1]
        states.append(state)
        velocities.append(state[0])
        time.append(t_end)

    clean_sectors={int(k):float(v) for k,v in sector_times.items()}
    for sector,t_sec in sorted(clean_sectors.items()):
        print(f"sector {sector}: {t_sec:.3f} seconds")
        time_s.append(t_sec)
    laptime=np.sum(time_s)

    print(f"Estimated Laptime: {laptime:.3f} seconds")
    return states, velocities, time, laptime,time_s, ax_actual
    
def simulate_endurance(state,states,velocities,time,track,vehicle,human_factor,total_laps=22,ax_actual=0.0):
    total_time = time[-1] if time else 0.0
    lap_times = []
    all_sector_times = []

    for lap in range(1, total_laps + 1):
        print(f"\n--- Lap {lap} ---")
        states, velocities, time, laptime, sector_times ,ax_actual = simulate_lap(state, states, velocities, time, track, vehicle, human_factor,ax_actual)
        # Prepare state for next lap (continue from last state)
        state = states[-1]
        lap_times.append(laptime)
        all_sector_times.append(sector_times)

    total_time = time[-1]
    print(f"\n=== Endurance Simulation Complete ===")
    print(f"Total time for {total_laps} laps: {total_time:.3f} seconds")
    print(f"Average lap time: {np.mean(lap_times):.3f} seconds")
    
    return states, velocities, time, lap_times, all_sector_times

#%% results
def extract_text_from_pdf(pdf_path):
  tables=[]
  with pdfplumber.open(pdf_path) as pdf:
      for page in pdf.pages:
          tables_on_page=page.extract_tables()
          tables.extend(tables_on_page)
  return tables

def extract_clean_times(table_idx, col_idx, skip_rows,extracted_tables):
    try:
        table = extracted_tables[table_idx]
        col = [row[col_idx] for row in table if len(row) > col_idx]
        col = col[skip_rows:]  # Skip headers or extra rows
        # Replace "DNA" with a high dummy value
        cleaned = [100000 if x == "DNA" else float(x) for x in col]
        return cleaned
    except IndexError as e:
        print(f"Error extracting table {table_idx}, column {col_idx}: {e}")
        return []

def acceleration_points(min, your):
  '''Takes the minimum time and your time and returns the points recieived for the acceleration event'''

  max=1.5*min
  if your>max:
    acc_points=4.5
    return acc_points

  acc_points=95.5*((max/your)-1)/((max/min)-1)
  acc_points+=4.5
  return acc_points

def skidpad_points(min,your):
  '''Takes the minimum time and your time and returns the points recieved for the skidpad event'''
  max=1.25*min
  if your>max:
    skid_points=3.5
    return skid_points
  skid_points=71.5*((max/your)**2-1)/((max/min)**2-1)
  skid_points+=3.5
  return skid_points

def autocross_points(min,your):
  '''Takes the minimum time and your time and returns the points recieved for the autocross event'''
  max=1.45*min
  if your>max:
    auto_points=6.5
    return auto_points
  auto_points=118.5*((max/your)-1)/((max/min)-1)
  auto_points+=6.5
  return auto_points

def endurance_max_points(min,your):
  '''Takes the minium total time and your single lap time and returns the max amount of points received for the endurance event.
  Assuming consistent time for each lap, and no penalties'''
  your=np.sum(your)
  max=1.45*min
  if your>max:
    end_points=25 #this is only true if you complete all laps. Otherwise it is the amount of laps completed +2
    return end_points
  end_points=250*((max/your)-1)/((max/min)-1) #score for time
  end_points+=25 #score for completing the laps. Again only 25 if all laps are completed
  return end_points


def efficiency_factor(min_time,your_time,min_energy,your_energy):
  '''takes the min time, your time, min energy, and your energy and returns the efficiency factor for the efficiency event'''
  max_time=1.45*min_time
  max_energy=1.25*min_energy

  lap_min=22 #assuming fully completed endurance
  Co2_min=min_energy*.65 #converts kWh to Kg

  Co2_yours=your_energy*.65 #converts kWh to Kg
  lap_yours=22 #assuming fully completed endurance

  factor=(min_time/lap_min)/(your_time/lap_yours) * (Co2_min/lap_min)/(Co2_yours/lap_yours)
  factor=round(factor,3)
  return factor

def efficiency_points(min_time,your_time,min_energy,your_energy):
  '''takes the min time, your time, min energy, and your energy and returns the points recieved for the efficiency event'''

  your=efficiency_factor(min_time,your_time,min_energy,your_energy)
  print(your)
  max=efficiency_factor(min_time,85.093,min_energy,min_energy)
  #the time listed here is simply the correlating laptime of the highest efficiency

  max_energy=22*.2002*1.54
  min=efficiency_factor(min_time,1.45*min_time,min_energy,max_energy)

  eff_points=100*(your-min)/(max-min)
  eff_points=round(eff_points,1)
  return eff_points

def calculate_result(track,track_config,extracted_tables,laptime):
    config = track_config.get(track.track_type)
    if not config:
        raise ValueError(f"Unknown track type: {track.track_type}")
    
    page, column, skip_rows = config["params"]
    times = extract_clean_times(page, column, skip_rows, extracted_tables)
    func=config["points_func"]
    points=func(times[0],laptime)

    place=1
    for i in times:
       if laptime> i:
          place+=1
       else:
          break
    return points, config["max_points"], place

def plot_simulation_results(track, velocities, sector_limits, sector_times, points, max_points, place):
    """
    Plots the velocity profile along the track and the track layout
    with velocity colormap.

    Args:
        s: Array of cumulative track distances.
        velocities: List or array of vehicle velocities.
        x: Array of track x-coordinates.
        y: Array of track y-coordinates.
        ti: List or array of simulation time points.
    """
    s=track.s
    x=track.x
    y=track.y
    
    fig, axs = plt.subplots(1,3,figsize=(13,5),gridspec_kw={'width_ratios':[5,1,5]})
    Batery=100
    Temp=20

    # --- Subplot 1: Velocity Profile ---
    axs[0].plot(s, velocities, color='green')
    axs[0].set_title('Velocity Profile along the Track')
    axs[0].set_xlabel('Track Position (m)')
    axs[0].set_ylabel('Speed (m/s)')
    axs[0].grid(True)
    for i,boundary in enumerate(sector_limits):
        axs[0].axvline(x=boundary, color='red',linestyle='--',linewidth=1)
        axs[0].text(boundary-25,min(velocities),f'Sector {i}',rotation=90)
        axs[0].text(boundary-25,max(velocities)*.95,f'{sector_times[i]:.2f}',rotation=90)

    # --- Subplot 2: Track Layout with Velocity Colormap ---
    norm = plt.Normalize(min(velocities), max(velocities))
    cmap = plt.cm.viridis
    sc = axs[2].scatter(x, y, c=velocities, cmap=cmap, norm=norm, s=5)
    axs[2].set_title('Track Layout with Velocity Profile')
    axs[2].set_xlabel('X (m)')
    axs[2].set_ylabel('Y (m)')
    for i,boundary in enumerate(sector_limits):
        idx=(np.abs(s-boundary)).argmin()
        axs[2].plot(x[idx],y[idx],marker='o',color='red',markersize=6)
        axs[2].text(x[idx],y[idx],i)

    # Add colorbar and summary text
    fig.colorbar(sc, ax=axs[2], label='Speed (m/s)')

    summary_text=f"""Lap time:{np.sum(sector_times):.2f} s
    Top Speed: {max(velocities):.2f} m/s
    Average Speed: {np.mean(velocities):.2f} m/s"""
    axs[1].axis('off')
    fig.text(0.5,.5, summary_text,fontsize=10,ha='center',va='center',bbox=dict(facecolor='white',edgecolor='black'))

    battery_text=f"""Bat. Useage :{Batery} kw
    Max Temperature :{Temp:.2f} C """
    fig.text(.5,.75,battery_text, fontsize=10,ha='center',va='center',bbox=dict(facecolor='white',edgecolor='black'))

    placement_text=f"""Expected Place :{place}
    Points: {points:.2f}/{max_points}"""
    fig.text(.5,.25,placement_text, fontsize=10,ha='center',va='center',bbox=dict(facecolor='white',edgecolor='black'))
    plt.tight_layout()
    plt.show()
