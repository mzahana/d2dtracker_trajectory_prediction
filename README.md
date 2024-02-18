# d2dtracker_trajectory_prediction
A ROS 1 package with nodes that implement real-time drone trajectory predicitons using adaptive model selection and evaluation techniques. 

![D2DTracker System Architecture](/images/d2dtracker_system_architecture.png "D2DTracker System Architecture")

**NOTE**

**This repository is part of the D2DTracker work which is submitted for publication. The code will be availble once the paper is accepted.**

# Installation
This package is tested on Ubuntu 22 with ROS Noetic. The following dependencies are required
* Eigen
* [osqpEigen](https://github.com/robotology/osqp-eigen.git)
* [custom_trajectory_msgs](https://github.com/mzahana/custom_trajectory_msgs.git)

Clone this package in your `catkin_ws/src` and compile using `catkin build`.

# Run
* An example launch file is available in `launch/predictor.launch`
* We recommend to use the [PX4 autopilot](https://github.com/robotology/osqp-eigen.git) + MAVROS to simulate UAVs in Gazebo and ROS.
* You can use our implementaiton of [multi-target Kalman filter](https://github.com/mzahana/multi_target_kf.git) to estimate the target's states.