# sdl-packages

This repository holds code for the SDL simulation project at Argonne National Laboratory/University of Chicago

## Setup
Assuming ROS Melodic is installed.

(DEPRECATED. 6 August 2021 No need to install git lfs) Make sure git LFS is installed. Documentation for git lfs can be found [here](https://git-lfs.github.com/)

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
git clone https://github.com/dsquez/sdl-application.git sdl_application
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
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
`python sdl_gazebo/python-nodes/can_demo.py`

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
  
The main launch file, `sdl_gazebo/launch/sdl_nav_moveit_combined.launch`, makes a call to `sdl_gazebo/src/sdl_moveit_node.py`, which starts up a ROS node that subscribes to `/ur_arm/moveit/goal_pose`. When this topic is published to, the Python code uses MoveIt to control the arm. 

The MoveIt configuration for the arm is `sdl_moveit_config`. It was created with the help of the MoveIt setup assistant.

The URDF is obtained [here](https://github.com/fmauch/universal_robot/tree/calibration_devel)

Basic joint control of the arm is implemented through the `limb.py` class in `sdl_interface`. As of 6 August 2021, the `set_joints_position()` function works to publish joint commands to gazebo. This is demonstrated by the following launch command:

`roslaunch sdl_application test_limb_class.launch`

You will get errors that the velocity is not specified. This does not seem to effect functionality.

Other functions are not tested or implemented. TODO: `limb.py` has functions for end effector positioning. There is not an inverse kinematics implementation to use this.

### MIR Navigation

The MIR can be navigated by publishing to `/move_base_simple/goal`, where the starting position of the robot is (0, 0).

The MIR base uses this [library](https://github.com/dfki-ric/mir_robot) to map and navigate. The launch files used in this [tutorial](https://github.com/dfki-ric/mir_robot#gazebo-demo-mapping) are included in `sdl_gazebo/launch/sdl_nav_moveit_combined.launch`. 

To see the map in RVIZ, which is launched by default, view the bottom left corner of the grid. See the MIR library for more information.

### 2F140 Gripper Control

The 2F140 Gripper (found [here](https://github.com/ros-industrial/robotiq)) is controlled by publishing to `/left_gripper_position_controller/command` and `/right_gripper_position_controller/command`. 

### Microsoft Kinect Camera

The Microsoft Kinect uses the `libgazebo_ros_openni_kinect.so` plugin to simulate sensor data. Point cloud data is published to `/camera/depth/points`. See `sdl_robot_description/urdf/camera.urdf.xacro` for more information.

### Ar Tag Recognition

This [library](http://wiki.ros.org/ar_track_alvar) is used to track AR tags. AR tag location is published to `/ar_pose_marker`. The AR tracking node is started in `sdl_gazebo/launch/ar_camera.launch`.

### General Information

Controllers are started up in `sdl_moveit_config/launch/ros_controllers.launch`. 

The MIR controllers are loaded in `sdl_gazebo/launch/mir_gazebo_common.launch`. 

The main URDF file is `sdl_robot_description/urdf/sdl_robot.urdf.xacro`, which is spawned into Gazebo in `sdl_gazebo/launch/mir_gazebo_common.launch`. 

The world file, `sdl_gazebo/world/lab_ar.world`, is loaded in `sdl_gazebo/launch/lab_world.launch`.

## Useful Links
[Creating Moveit config file](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)

[Depth Camera integration](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros)

[ROS plugin list](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros)

[YAML format for map](https://wiki.ros.org/map_server)

[Saving Gazebo world bug](https://stackoverflow.com/a/67088987)

[Creating AR Tags](https://github.com/mikaelarguedas/gazebo_models)
