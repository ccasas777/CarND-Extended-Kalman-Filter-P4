# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, I utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. In the project rubric, I need to acheive the obtained RMSE values that are lower than the tolerance . 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

# Build in src

1. main.cpp
2. FusionEKF.cpp
3. kalman_filter.cpp
4. tool.cpp

main.cpp - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function
kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar
tools.cpp- function to calculate RMSE.


How the Files Relate to Each Other:
Main.cpp reads in the data and sends a sensor measurement to FusionEKF.cpp
FusionEKF.cpp takes the sensor data and initializes variables and updates variables. 
FusionEKF.cpp has a variable called ekf_, which is an instance of a KalmanFilter class. 
The ekf_ will hold the matrix and vector values. You will also use the ekf_ instance to call the predict and update equations.
The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h. 

# RUN the Project:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

# DEMO video: https://youtu.be/Ge-iiLgHQFs


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---
# The final result
![Image1](./IMAGE/image1.png)



