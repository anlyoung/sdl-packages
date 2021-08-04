# sdl-packages

This repository holds code for the SDL simulation project at Argonne National Laboratory/University of Chicago

## Setup
Assuming ROS Melodic is installed.

Make sure git LFS is installed. Documentation for git lfs can be found [here](https://git-lfs.github.com/)

`git lfs install`

Install the AR library:

`sudo apt-get install ros-melodic-ar-track-alvar`

Install MoveIt:

`sudo apt-get install ros-melodic-moveit`

Create a workspace for the project:

`mkdir -p catkin_ws/src`

Go into the workspace and clone the repository:
```
cd catkin_ws/src
git clone https://github.com/dsquez/sdl-packages.git
```

Clone other repositories that we're using
```
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
git clone -b melodic https://github.com/dfki-ric/mir_robot.git
git clone https://github.com/ros-industrial/robotiq.git
```

Go back to workspace base, install dependencies, and build:
```
cd ..
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make
```

Source the new workspace:

`source catkin_ws/devel/setup.bash`

## Running

Run the launch file:

`roslaunch sdl_gazebo sdl_nav_moveit_combined.launch`

The gazebo window is launched in a paused state to give the controllers enough time to initialize, when you see the message:

`[ WARN] [1627157134.932762011]: service '/get_planning_scene' not advertised yet. Continue waiting...
[ INFO] [1627157134.933636210]: waitForService: Service [/get_planning_scene] has not been advertised, waiting... `

**Press the play button in Gazebo.** You will see the arm oscillate slightly. This can be mitigated in the future with controller gain tuning.

When you see the message:

`You can start planning now!`

`[ INFO] [1627157159.619626329, 18.202000000]: Ready to take commands for planning group ur_arm.`

You are ready to proceed.

### Demo Picking Up Can
`python sdl_gazebo/python-nodes/can_demo_no_camera.py`

## Details

### UR5E Arm Control

The following is an example of publishing to the topic to control the arm:

`rostopic pub -1 /ur_arm/moveit/goal_pose geometry_msgs/Pose "position:
  x: 1.2
  y: 0.0
  z: 1.5
orientation:
  x: 0.0
  y: 0.707
  z: 0.0
  w: 0.707"`
  
The launch file `sdl_gazebo/launch/sdl_nav_moveit_combined.launch` makes a call to `sdl_gazebo/src/sdl_moveit_node.py`, which starts up a ROS node that subscribes to `/ur_arm/moveit/goal_pose`. When this topic is published to, the Python code uses MoveIt to control the arm. 

The MoveIt configuration is `sdl_moveit_config`, and it is started up in `sdl_gazebo/launch/sdl_nav_moveit_combined.launch`.

The URDF is obtained [here](https://github.com/fmauch/universal_robot/tree/calibration_devel)

### MIR Navigation

The MIR can be navigated by publishing to `/move_base_simple/goal`, where the starting position of the robot is (0, 0).

The MIR base uses this [library](https://github.com/dfki-ric/mir_robot) to map and navigate. The launch files used in this [tutorial](https://github.com/dfki-ric/mir_robot) are launched in `sdl_gazebo/launch/sdl_nav_moveit_combined.launch`. 

To see the map in RVIZ, which is launched by default, view the bottom left corner of the grid. See the MIR library for more information.

### 2F140 Gripper Control

The 2F140 Gripper (found [here](https://github.com/ros-industrial/robotiq)) is controlled by publishing to `/left_gripper_position_controller/command` and `/right_gripper_position_controller/command`. 

### Microsoft Kinect Camera

The Microsoft Kinect uses the `libgazebo_ros_openni_kinect.so` plugin to simulate sensor data. Point cloud data is published to `/camera/depth/points`. See `sdl_robot_description/urdf/camera.urdf.xacro` for more information.

### Ar Tag Recognition

This [library](http://wiki.ros.org/ar_track_alvar) is used to track AR tags. AR tag location is published to `/ar_pose_marker`. The AR tracking node is started in `sdl_gazebo/launch/ar_camera.launch`, which is included in `sdl_gazebo/launch/sdl_nav_moveit_combined.launch`.

### General Information

Controllers are started up in `sdl_moveit_config/launch/ros_controllers.launch`. The MIR controllers are loaded in `sdl_gazebo/launch/mir_gazebo_common.launch`. The main URDF file is `sdl_robot_description/urdf/sdl_robot.urdf.xacro`, which is spawned into Gazebo in `sdl_gazebo/launch/mir_gazebo_common.launch`. The world file, `sdl_gazebo/world/lab_ar.world`, is loaded in `sdl_gazebo/launch/lab_world.launch`.

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

[YAML format for map](https://wiki.ros.org/map_server)

[Saving Gazebo world bug](https://stackoverflow.com/a/67088987)

[2F140 gripper](https://github.com/ros-industrial/robotiq)

[Creating AR Tags](https://github.com/mikaelarguedas/gazebo_models)