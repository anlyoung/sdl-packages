# sdl-packages

This repository holds code for the SDL simulation project at Argonne National Laboratory/University of Chicago. It provides controllers and robot models that allow users to create simulations.

## Setup
Assuming ROS Melodic is installed.

Install the AR library:

`sudo apt-get install ros-melodic-ar-track-alvar`

Install MoveIt:

`sudo apt-get install ros-melodic-moveit`

Create a workspace for the project:

`mkdir -p catkin_ws/src`

Go into the workspace and clone the repository:
```
cd catkin_ws/src
git clone https://github.com/anlyoung/sdl-packages.git
```

Clone other repositories that we're using
```
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
git clone -b melodic https://github.com/dfki-ric/mir_robot.git
git clone https://github.com/ros-industrial/robotiq.git
git clone https://github.com/anlyoung/sdl-application.git sdl_application
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
```

Move Gazebo models:
```
cp sdl-packages/sdl_gazebo/models/ar_box ~/.gazebo/models -r
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

### Demo of moveit planning
Run the launch file:

`roslaunch sdl_gazebo sdl_nav_moveit_combined.launch`

The gazebo window is launched in a paused state to give the controllers enough time to initialize, when you see the message:

`[ WARN] [1627157134.932762011]: service '/get_planning_scene' not advertised yet. Continue waiting...
[ INFO] [1627157134.933636210]: waitForService: Service [/get_planning_scene] has not been advertised, waiting... `

**Press the play button in Gazebo.** You will see the arm oscillate slightly. This can be mitigated in the future with controller gain tuning.

When you see the message:

`You can start planning now!`

`[ INFO] [1627157159.619626329, 18.202000000]: Ready to take commands for planning group ur_arm.`

You are ready to proceed. You can run the following python script or publish to the `/ur_arm/moveit/goal_pose` topic

### Demo Picking Up Can
`python sdl_gazebo/src/can_demo.py`

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

MoveIt is used to control the arm. The configuration `sdl_moveit_config` was created with the help of the MoveIt setup assistant.
  
MoveIt is started by running `roslaunch sdl_moveit_config move_group.launch`
  
`sdl_gazebo/src/sdl_moveit_node.py` starts up a ROS node that subscribes to `/ur_arm/moveit/goal_pose`. When this topic is published to, the Python code uses MoveIt to control the arm. 

The URDF of the arm is found here: `$(find sdl_robot_description)/urdf/inc/ur5e_macro.xacro`([source](https://github.com/fmauch/universal_robot/tree/calibration_devel)).

Basic joint control of the arm is implemented through the `limb.py` class in `sdl_interface`. 

Other functions are not tested or implemented. TODO: `limb.py` has functions for end effector positioning. There is not an inverse kinematics implementation to use this.

### MIR Navigation

The MIR base uses this [library](https://github.com/dfki-ric/mir_robot) to map and navigate. The URDF is in the file `$(find mir_description)/urdf/mir.urdf.xacro`. The controllers are `$(find mir_description)/config/joint_state_controller.yaml` and `$(find mir_description)/config/diffdrive_controller.yaml`.

To enable mapping and navigation, the following must be launched:
```
roslaunch mir_navigation hector_mapping.launch
roslaunch mir_navigation move_base.xml with_virtual_walls:=false
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```

The MIR can be navigated by publishing to `/move_base_simple/goal` or by setting a goal in RVIZ, where the starting position of the robot is (0, 0).

See the MIR library for more information.

### 2F140 Gripper Control

The 2F140 Gripper ([source](https://github.com/ros-industrial/robotiq)) is controlled by publishing to `/gripper_position_controller/command`. Its controller is in `sdl_gazebo/config/gripper_control.yaml` and its URDF is in `sdl_robot_description/urdf/2f140.urdf.xacro`.

### Microsoft Kinect Camera

The Microsoft Kinect uses the `libgazebo_ros_openni_kinect.so` plugin to simulate sensor data. Point cloud data is published to `/camera/depth/points`. The URDF is found here: `sdl_robot_description/urdf/camera.urdf.xacro`.

The Kinect sensor input can be visualization by running RVIZ with the configuration `$(find sdl_robot_description)/cfg/point_cloud_config.rviz`.

### Ar Tag Recognition

This [library](http://wiki.ros.org/ar_track_alvar) is used to track AR tags. AR tag location is published to `/ar_pose_marker`. To start the AR tracking node, launch`sdl_gazebo/launch/ar_camera.launch`. Make sure to set the marker_size parameter (in centimeters).

## Useful Links
[Creating Moveit config file](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)

[Depth Camera integration](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros)

[ROS plugin list](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros)

[YAML format for map](https://wiki.ros.org/map_server)

[Saving Gazebo world bug](https://stackoverflow.com/a/67088987)

[Creating AR Tags](https://github.com/mikaelarguedas/gazebo_models)
