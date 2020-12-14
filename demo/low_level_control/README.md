## to run:

0. all the packages run on ROS kinetic on Ubuntu 16.04. running the simulation requires the turtlebot metapackage, `turtlebot3_simulation`, which requires the `turtlebot3` metapackage and `turtlebot3_msgs` packages. see [Link](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) for more info and download instructions. all 3 packages/metapackages should be in `~/catkin_ws/src`

1. put the package into `~/catkin_ws/src`

2. ensure `ROS_MASTER_URI` is set in both `~/.bashrc` and the matlab file. e.g. in bashrc:

```
export ROS_MASTER_URI=http://$(hostname -I | cut -d ' ' -f1):11311
export ROS_HOSTAME=$(hostname -I | cut -d ' ' -f1)
```

in matlab, pass `ROS_MASTER_URI` as a string into the `rosinit` function (replace line 7 in the matlab file)

### to run in simulation

1. the turtlebot used is waffle, so run `export TURTLEBOT3_MODEL=waffle` or add it to the bashrc file

2. start ROS, run the matlab script, which will publish the trajectory and size of timestep (topic is /dt), and run 

```
roslaunch low_level_control follow_traj.launch
```
or


1. change human movement by editing velocities in fake_human4.py

```
roslaunch low_level_control follow_fake_human_withbarrier.launch
```

