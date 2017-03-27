# Extended_Kalman_Filter_Project

There are 4 applications in this repository:
- Jacobian
- KalmanFilter
- KalmanFilter2D
- Extended_Kalman_Filter_Project

The Extended_Kalman_Filter_Project implements a full EKF using both Laser and Radar data.

# Dependencies:
cmake >= 3.5

# Compiling instructions:
Make a build directory: mkdir build && cd build
Compile: cmake .. && make
On windows, you may need to run: cmake .. -G "Unix Makefiles" && make
Run it: ./ExtendedKF path/to/input.txt path/to/output.txt. You can find some sample inputs in 'data/'.
eg. ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt 
