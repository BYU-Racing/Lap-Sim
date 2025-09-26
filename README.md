# Lap-Sim
The lap sim currently operates on a modified bicycle model. Accounting for 4 different slip angles, and front/back load transfer.It also accounts for continuously changing cornering stiffness, aerodynamics, and motor torque. Battery Usage and Regen are being tested. 

Before you can run the code, you will need the CSVs provided in the repo,or you can make your own following the data as templates.
You will need atleast 1 track, the cornering stiffness, and the motor vs. torque in order to simulate a lap. In order to calculate points and/or placement you will also need the competition results. 

The vehicle parameters are included in the vehicle setup file. The parameters included are for the 2025 car. Any changes to those would currently need to be hard coded into the vehicle setup. I am working on defining them in a slightly different way to allow easier chanigng of parameters. 

I have included the option for sectors within a track. These will need to be defined as a start and end point in order for the visualization to work properly. 
Ex. (0.0,150 , 300) would make 2 sectors one from 0 to 150 and another from 150 to 300.
The system can handle as many as you want but very small sectors aren't very useful. Additionally you could put no sectors and the code would work just fine.

The simulation currently will simulate 1 lap of the endurance track and then attempt to simulate multiple laps of anything else. If you would like to switch that simply change the == to a !=
With each simulation it will print any sector times as well as the final laptime. There is also a plot_simulation_results function to display a more detailed account of the simulation.
I have also included some code to compare 2 or more simulations, although the results only really compare the velocites. 

The human factor is a little confusing but essentially the value (can be anything from .000000001 to 1) determmines the minimum percentage of the acceleration / braking delay. The simulation will then randomly choose a number between that minimum and the max (1) to determine the delay. 
ex. human_factor=1 would be a perfect simulation, with perfect acceleration/ braking. While human_factor=.5 would be a simulation that assumes the driver is atleast able to match 50% of perfect acceleration/braking.

There are a couple of debugging functions inlcuded in the run simulation file. Generally speaking these things shouldnt be an issue, but just to double check some things. 
