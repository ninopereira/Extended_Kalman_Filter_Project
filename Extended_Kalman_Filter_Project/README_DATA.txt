The data in "sample-laser-radar-measurement-data-1.txt" is organised as follows

Each line contains a measurement for a give type of sensor
The first string (which is a letter in this case) tell if the measurement refers to LIDAR or RADAR data

For the LIDAR the following data is registered:
# x, y, timestamp, ground_truth x, ground_truth y, ground_truth vx, ground_truth vy

For the RADAR the following data is registered:
# ro, phi, ro_dot, timestamp, ground_truth x, ground_truth y, ground_truth vx, ground_truth vy

