# Lap-Sim
The lap sim currently operates on a bicyle model. Assuming the same slip angle ,2 wheels, and no load transfer. It does however account for continuously changing cornering stiffness, aerodynamics, and motor torque. 
I am working on transitioning to a car model to allow for different slip angles and load transfer

In order to run it you will need a track, motor data, and cornering stiffness CSVs. I have provided them in the repo,or you can make your own following the data as templates.

The track will display before the simulation runs. You must close that out before the simulation will calculate the laptime.

The laptime is printed out before a velocity profile and a speed track map are displayed. 
I am working on including additional information with those as well. 
