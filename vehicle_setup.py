import numpy as np
import pandas as pd
from simulation_functions import *

class CarData:
    def __init__(self):
        #general not expected to change
        self.g =9.81 #gravity (m/s^2)
        self.mu=1.6 #friction Coefficinet
        self.rho=1.2255 #air density at STP
        self.C_rr=0.015 #rolling resistance coefficinet
        self.tire_radius=0.203 #meters 
        self.ax_brake=1.4*self.g #max longitudianl braking
        self.ay_max_limit=1.5*self.g #max lateral load allowed from tires
        self.m=200 #mass (kg)

class Suspension:
    def __init__(self):
        self.a=0.816
        self.b=0.823 # distance from Center of Gravity to front and rear axles (m)
        self.l=self.a + self.b #total wheel base (m)
        self.track_width=1.25 #width of center to center tires (from front)
        self.Iz=85.2 #moment of inertia in the roll direction
        self.h=0.218 #ride hieght (m)

        # --- Cornering Stiffness ---
        data = np.array(pd.read_csv('cornering_stiffness_vs_latg.csv'))
        latg_data= data[:, 0]
        Front_data = data[:, 1]
        Rear_data = data[:, 2]
        #Fit front and rear cornering stiffness curves
        self.front_params = fit_cornering_stiffness(latg_data, Front_data)
        self.rear_params = fit_cornering_stiffness(latg_data, Rear_data)

class Aero:
    def __init__(self):
        self.A=2.224 #frontal area
        self.Cd=.6 #normal is .6 
        self.Cl=-3 #drag and lift coefficients. Negative Cl= downforce

class Motor:
    def __init__(self):
        self.gear_ratio=4.14 #final drive ratio
        motor_data = pd.read_csv('208 motor data.csv')
        self.params, self.model = fit_motor_curve(motor_data) #find fit for motor data

class VehicleModel:
    def __init__(self,general:CarData,Suspension:Suspension, aero:Aero,motor:Motor):
        self.general=general
        self.suspension=Suspension
        self.aero=aero
        self.motor=motor

class TrackData:
    def __init__(self, filename:str,vehicle:VehicleModel,v_max_straight:float =100.0,tolerance=2.5e-10
                 ,sector_boundaries=None):
        self.filename=filename
        self.track_type=detect_track_type(self.filename)
        self.vehicle=vehicle
        self.mu=vehicle.general.mu
        self.g=vehicle.general.g
        self.ay_max=vehicle.general.ay_max_limit
        self.v_max_straight=v_max_straight
        self.tolerance=tolerance
        self.sector_boundaries=sector_boundaries
        self._load_track()
        self._compute_vmax_profile()

    def _load_track(self):
        self.track=pd.read_csv(self.filename)
        self.s,self.ds,self.curvature,self.x,self.y, self.sector_list =track_builder(self.track,self.sector_boundaries)

    def _compute_vmax_profile(self):
        self.v_max_profile = []
        ay_max = min(self.mu * self.g, self.ay_max)
        for k in self.curvature:
            if abs(k) < self.tolerance:
                self.v_max_profile.append(self.v_max_straight)
            else:
                self.v_max_profile.append(np.sqrt(ay_max / abs(k)))

#%% Results setup 
extracted_tables = extract_text_from_pdf("fsae_ev_2024_results.pdf")

track_config = {
    "accel": {
        "params": (10, 15, 2), #these numbers represent the page number, coulmn of data to record and any rows to skip. They will need to be adjusted if using a different results pdf
        "points_func": acceleration_points,
        "max_points": 100
    },
    "skid": {
        "params": (13, 19, 2),
        "points_func": skidpad_points,
        "max_points": 75
    },
    "auto": {
        "params": (15, 19, 2),
        "points_func": autocross_points,
        "max_points": 125
    },
    "endur": {
        "params": (18, 8, 1),
        "points_func": endurance_max_points,
        "max_points": 275
    }
}
