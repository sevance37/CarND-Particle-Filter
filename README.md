# CarND-Particle-Filter
Estimate the location of a moving object by using a particle filter.  
The program is written in C++.  This Project is from Udacity's Self-Driving Car Engineer Nanodegree Program.

## Basic Set-up
1. Clone this repo.
2. Make sure [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) is installed.  Two install scripts are included for MAC and Linux.  For Windows use [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) and following the Linux instructions.
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run the programs: Run `./particle_filter`. Open [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) and run the corresponding page.

## Program Input and output.
INPUT: values provided by the simulator to the c++ program  
// sense noisy position data from the simulator  
["sense_x"]  
["sense_y"]  
["sense_theta"]  
// get the previous velocity and yaw rate to predict the particle's transitioned state  
["previous_velocity"]  
["previous_yawrate"]  
// receive noisy observation data from the simulator, in a respective list of x/y values  
["sense_observations_x"]  
["sense_observations_y"]  

OUTPUT: values provided by the c++ program to the simulator  
// best particle values used for calculating the error evaluation  
["best_particle_x"]  
["best_particle_y"]  
["best_particle_theta"]  
//Optional message data used for debugging particle's sensing and associations  
// for respective (x,y) sensed positions ID label  
["best_particle_associations"]  
// for respective (x,y) sensed positions  
["best_particle_sense_x"]  
["best_particle_sense_y"]  

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.  The file `map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
