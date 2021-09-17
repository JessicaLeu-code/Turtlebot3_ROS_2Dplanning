# Turtlebot3_ROS_2Dplanning

This repo contains a simulation example of [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) motion planning in environment with moving obstacles. 


## Get ready
The setup is tested with Ubuntu 18.04. (Windows user can setup an Ubuntu environmnet by following [these instructions](https://youtu.be/IL7Jd9rjgrM).)To install dependent packages, folloe the instructions in 
1. [Setup turtlebot3 on PC](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
2. [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation). 
3. Clone this repository in your ``<your_dir>/catkin_ws``.
    ```
    cd ~/<your_dir>/catkin_ws && git clone https://github.com/JessicaLeu-code/Turtlebot3_ROS_2Dplanning.git
    ```
5. Make modifications
  This Demo is based on old version of turtlebot3 open-source code. Therefore, some addition files are needed to run the demo.
    - remove `turtlebot3_simulations/turtlebot3_gazebo`:
      ```
      cd ~/<your_dir>/catkin_ws/src/urtlebot3_simulations && sudo rm -r turtlebot3_gazebo
      ```
    - Download ``turtlebot3`` dependencies: 
      ```
      cd ~/<your_dir>/catkin_ws/src && clone https://github.com/ROBOTIS-GIT/turtlebot3.git
      ```
6. Install packages
  - Install (update) basic packages: ``python -m pip install --user numpy scipy matplotlib``
  - Install pyclipper: ``pip install pyclipper`` (you may need `pip install --upgrade setuptools` if am error occures)
  - Install pykalman: ``pip install pykalman``
  - Install cvxopt: ``pip install cvxopt``
7. Build the packages: ``cd ~<your_dir>\catkin_ws && catkin_make``

## Run the example

Open a terminal, copy and run the following command.

`$ roslaunch lidar_track turtlebot3_lidar_track_mode.launch`

To change the goal of the robot, run

`$ roslaunch demo_teleop turtlebot3_teleop_key.launch`

![GitHub Logo](/pic/2d_demo.gif)
