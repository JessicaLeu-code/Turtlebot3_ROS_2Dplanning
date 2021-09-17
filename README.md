# Turtlebot3_ROS_2Dplanning

This repo contains a simulation example of [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) motion planning in environment with moving obstacles. 


## Get ready
The setup is tested with Ubuntu 18.04. (Windows user can setup an Ubuntu environmnet by following [these instructions](https://youtu.be/IL7Jd9rjgrM).)To install dependent packages, folloe the instructions in 
1. [Setup turtlebot3 on PC](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
    ```
    # Install ROS (melodic for Ubuntu 18.04)
    $ sudo apt update
    $ sudo apt upgrade
    $ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
    $ chmod 755 ./install_ros_melodic.sh 
    $ bash ./install_ros_melodic.sh
    
    # Install Dependent ROS Packages
    $ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
      ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
      ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
      ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
      ros-melodic-rosserial-server ros-melodic-rosserial-client \
      ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
      ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
      ros-melodic-compressed-image-transport ros-melodic-rqt* \
      ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
      
    # Install TurtleBot3 via Debian Packages
    $ sudo apt-get install ros-melodic-dynamixel-sdk
    $ sudo apt-get install ros-melodic-turtlebot3-msgs
    $ sudo apt-get install ros-melodic-turtlebot3
    
    # Additional setup
    echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc 
    ```
2. [Install Simulation Package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation). 
    ```
    $ cd ~/<your_dir>/catkin_ws/src/
    $ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    $ cd ~/<your_dir>/catkin_ws && catkin_make
    ```
    Now you can test the installation with command: `$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
3. Clone this repository in your ``<your_dir>/catkin_ws``.
    ```
    $ cd ~/<your_dir>/catkin_ws && git clone https://github.com/JessicaLeu-code/Turtlebot3_ROS_2Dplanning.git
    ```
4. Make modifications
  This Demo is based on old version of turtlebot3 open-source code. Therefore, some addition files are needed to run the demo.
    - remove `turtlebot3_simulations/turtlebot3_gazebo`:
      ```
      $ cd ~/<your_dir>/catkin_ws/src/urtlebot3_simulations && sudo rm -r turtlebot3_gazebo
      ```
    - Download ``turtlebot3`` dependencies: 
      ```
      $ cd ~/<your_dir>/catkin_ws/src && clone https://github.com/ROBOTIS-GIT/turtlebot3.git
      ```
5. Install packages
  - Install ROS related packages:
    - `$ sudo apt-get install ros-melodic-driver-base`
    - `$ sudo apt-get install ros-melodic-pol-ros`
    - `$ sudo apt-get install ros-melodic-tf2-sensor-msg`
  - Install libcurses5-dev`$ sudo apt-get install libcurses5-dev`
  - Install (update) basic packages: ``$ python -m pip install --user numpy scipy matplotlib``
  - Install pyclipper: ``$ pip install pyclipper`` (you may need `$ pip install --upgrade setuptools` if am error occures)
  - Install pykalman: ``$ pip install pykalman``
  - Install cvxopt: ``$ pip install cvxopt``
6. Build the packages: ``$ cd ~<your_dir>\catkin_ws && catkin_make``

## Run the example

Open a terminal, copy and run the following command.

```
$ roslaunch lidar_track turtlebot3_lidar_track_mode.launch
```

To change the goal of the robot, run

```
$ roslaunch demo_teleop turtlebot3_teleop_key.launch
```

![GitHub Logo](/pic/2d_demo.gif)
