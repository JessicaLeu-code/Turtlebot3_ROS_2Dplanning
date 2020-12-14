# Turtlebot3_ROS_2Dplanning

This repo contains a simulation example of [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) motion planning in environment with moving obstacles. 


## Get ready

To install dependent packages, folloe the instructions in [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation). 
run `RRT_planning/RRT_3_test.m` to plan for the mobile platform with RRT-ILQR.


## Run example

Open a terminal, copy and run the following command.
`$ roslaunch lidar_track turtlebot3_lidar_track_mode.launch`
To change the goal of the robot, run
`$ roslaunch demo_teleop turtlebot3_teleop_key.launch`
![GitHub Logo](/pic/2d_demo.gif)
