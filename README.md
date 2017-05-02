# SelfDrivingCar P7 Unscented Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)  
Udacity CarND Term 2  - Project 2 
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Files in the `src` Folder
* `main.cpp` - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `ukf.cpp` - initializes the filter, calls the predict and update function, defines the predict and update functions
* `tools.cpp` - function to calculate RMSE

## Data
The data file we are using is the same from EKF. Again each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next columns are either the two lidar position measurements (x,y) or the three radar position measurements (rho, phi, rho_dot). Then comes the time stamp and finally the ground truth values for x, y, vx, vy, yaw, yawrate.


## Simulation Results

Input file: `obj_pose-laser-radar-synthetic-input.txt`.

### RMSE Outputs for the Sample Inputs

* Use both Radar and Lidar measurements  
The `px, py, vx, vy` output coordinates have an `RMSE = [0.0651648, 0.0605379, 0.533212, 0.544193]`.
* Use only Radar measurement
The `px, py, vx, vy` output coordinates have an `RMSE = [0.0651648, 0.0605379, 0.533212, 0.544193]`.
* Use only Lidar measurement
The `px, py, vx, vy` output coordinates have an `RMSE = [0.0651648, 0.0605379, 0.533212, 0.544193]`.

