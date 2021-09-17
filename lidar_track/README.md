# NSF-T3
-motion planning/ Tracking <br/>
"mpt3" is needed for running the demo code.\
Run "install_mpt3.m" for installing the package.

---

## to run:

0. all the packages run on ROS kinetic on Ubuntu 16.04. running the simulation requires the turtlebot metapackage, `turtlebot3_simulation`, which requires the `turtlebot3` metapackage and `turtlebot3_msgs` packages. see [Link](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) for more info and download instructions. all 3 packages/metapackages should be in `~/catkin_ws/src`

1. pull the lidar_track, mobile_manipulator, human_prediction, turtlebot3_simulations packages into `~/catkin_ws/src`

2. ensure `ROS_MASTER_URI` is set in both `~/.bashrc` and the matlab file. e.g. in bashrc:

```
export ROS_MASTER_URI=http://$(hostname -I | cut -d ' ' -f1):11311
export ROS_HOSTAME=$(hostname -I | cut -d ' ' -f1)
```

in matlab, pass `ROS_MASTER_URI` as a string into the `rosinit` function (replace line 7 in the matlab file)

### to run the Gazebo simulation with object following

1. the turtlebot used is waffle, so run `export TURTLEBOT3_MODEL=waffle` or add it to the bashrc file

2. installation of pykalman is required for the obstacle detection node
```
sudo easy_install pykalman
```

3. start ROS, and run 

```
roslaunch lidar_track turtlebot3_lidar_track_barrier.launch
```
4. if desired, the trajectory for the object in Gazebo can be modified in `~/turtlebot3_simulations/turtlebot3_gazebo/src/follow_barrier.py`


5. If driver_base is not installed:
```
git clone https://github.com/ros-drivers/driver_common.git

```
```
sudo apt-get install ncurses-dev

```



