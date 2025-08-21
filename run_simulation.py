from simulation_functions import *
from vehicle_setup import *

#intialize vehicle
#I have programmed in the 2025 car as the defaults. 
# Any changes would have to be called here or hard coded into the setup
general=CarData()
susp=Suspension()
aero=Aero()
motor=Motor()

vehicle=VehicleModel(general,susp,aero,motor)

#--- Track---
sector_boundaries=[0.0,300,400,500,700,1025] #when each sector starts/ends. Kind of a pain since you have to manually decide where they go.
# best to put zeros initially and then look at the results output to determine where to put your sectors.
# Must include start and end of entire track, otherwise visualization gets a little weird

track=TrackData('2024 Endurance.csv',vehicle,sector_boundaries=sector_boundaries)#or whatever the name of the track file is

#---Setup---
inital_state = np.array([0.0, 0.0, 0.0])  # Initial state [vx, vy, r (yaw rate)]
states = [inital_state]
velocities = [inital_state[0]]
time = [0]

#---Run Simulation---
human_factor=.75
if track.track_type == 'endur':
    states1, velocities1, time1, laptime1, sector_times,ax_actual = simulate_lap(
    inital_state, states, velocities, time, track, vehicle, human_factor)

    points,max_points,place=calculate_result(track,track_config,extracted_tables,laptime1)
    plot_simulation_results(track, velocities1, sector_boundaries, sector_times, points, max_points, place)

else:
    states1, velocities1, time1, laptime1, sector_times = simulate_endurance(
        inital_state, states,velocities,time,track,vehicle,human_factor)
    
    points,max_points,place=calculate_result(track,track_config,extracted_tables,np.sum(laptime1))

    print(f'Points Scored: {points:.2f}/{max_points}')
    print(f'Expected Place: {place}')


#%%---Comparing Results---

'''
state = np.array([0.0, 0.0, 0.0])  # Initial state [vx, vy, r (yaw rate)]
states = [state]
velocities = [state[0]]
time = [0]

human_factor=.5
states2, velocities2, time2, laptime2 = run_simulation_car(
    state, states, velocities, time, track, vehicle,human_factor
)

#---Laptime---
print(f"Estimated Laptime: {laptime2:.2f} seconds")

if laptime2>laptime1:
    print(f"1st simulation was faster by {laptime2-laptime1}")
else:
    print(f"2nd simulation was faster by {laptime1-laptime2}")

# --- Subplot 1: Velocity Profile ---
plt.plot(track.s,velocities1)
plt.plot(track.s,velocities2)
plt.title('Velocity Profile along the Track')
plt.xlabel('Track Position (m)')
plt.ylabel('Speed (m/s)')
plt.grid(True)
plt.savefig('human_vs_ideal.png')
plt.show()
'''
#%% debugging
'''
#Plot the motor fit.
plot_motor_fit(motor, params, model)

#plot the cornering stiffness fit
datasets=[(Suspension.latg_data,Suspension.Front_data),(Suspension.latg_data,Suspension.Rear_data)]
labels=['Front','Rear']
fit_params=[Suspension.front_params,Suspension.rear_params]
plot_cornering_stiffness(datasets,labels,fit_params)

# --- Track Visualization ---
plt.figure(figsize=(8,4))
plt.plot(track.x, track.y, label='Track Centerline', color='blue')
plt.title('Track Layout')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.show()
    '''
