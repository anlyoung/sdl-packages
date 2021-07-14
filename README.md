# sdl-packages

This repository holds code for the SDL simulation project at Argonne National Laboratory/University of Chicago

## Setup
Assuming ROS Melodic is installed.

Add ROS descriptions for the robots we are using

* Install the Universal Robots ROS Driver package
  ```
  source /opt/ros/melodic/setup.bash
  mkdir -p catkin_ws/src && cd catkin_ws
  git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
  git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
  sudo apt update -qq
  rosdep update
  rosdep install --from-paths src --ignore-src -y
  catkin_make
  source devel/setup.bash
  ```
* `sudo apt install ros-noetic-mir-robot`

Create a workspace for the project:

`mkdir -p catkin_ws/src`

Go into the workspace and clone the repository:

`cd catkin_ws/src`

`git clone https://github.com/dsquez/sdl-packages.git`

Go back to src and build:

`cd catkin_ws`

`catkin_make`

Source the new workspace:

`source catkin_ws/devel/setup.bash`

## Running
To see the rviz visualization, make sure ros and the workspace are sourced and run

`roslaunch sdl_robot_description view_sdl.launch`

To launch the gazebo simulaion, run

`roslaunch sdl_gazebo sdl_robot.launch`

### Controlling with Python
Once the gazebo simulation is running, execute

`python sdl_gazebo/python-nodes/demo.py`

Moveit is not currently working (7 July 2021)

## Components
UR5e Arm

[Robotiq Adaptive Robot Gripper, 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button): [URDF](https://github.com/Improbable-AI/airobot/blob/master/src/airobot/urdfs/ur5e_2f140_pybullet.urdf)

EPick Kit for e-Series from UR / 1 cup

Dual Robotiq Adaptive Robot Gripper Adapter

MiR200 (with two batteries)

MiR Autonomous Charger (24V)

MiR/UR Vention Mounting Module for UR5e, MiR200

## Useful Links
[Creating Moveit config file](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)

[Depth Camera integration](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros)

[ROS plugin list](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros)

[PointCloud2 message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
