# sdl-packages

This repository holds code for the SDL simulation project at Argonne National Laboratory/University of Chicago

## Setup
Assuming ROS Melodic is installed.

Make sure git LFS is installed. Documentation for git lfs can be found [here](https://git-lfs.github.com/)

`git lfs install`

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

## IMPORTANT GAZEBO INFORMATION
As of 16 July 2021, I have been having issues getting the gazebo world to show up properly. It has only worked with absolute filepaths and changing environment variables. To run gazebo, change the absolute filepaths found in `sdl_gazebo/world/lab_simple.world` to reflect your machine, and in the terminal from which you will launch gazebo, run:

`export GAZEBO_RESOURCE_PATH="<path-to-your-workspace>/src/sdl-packages/sdl_gazebo/world/"`

This will hopefully be fixed in the near future.

## Running

Run the launch file:

`roslaunch sdl_gazebo sdl_nav_moveit_combined.launch`

The gazebo window is launched in a paused state to give the controllers enough time to initialize, when you see the message:

`[ WARN] [1627157134.932762011]: service '/get_planning_scene' not advertised yet. Continue waiting...
[ INFO] [1627157134.933636210]: waitForService: Service [/get_planning_scene] has not been advertised, waiting... `

Press play to start gazebo. You will see the arm oscillate slightly. This can be mitigated in the future with controller gain tuning.

When you see the message:

`You can start planning now!`

`[ INFO] [1627157159.619626329, 18.202000000]: Ready to take commands for planning group ur_arm.`

You are ready to proceed.

### Demo Picking Up Can
`python sdl_gazebo/python-nodes/can_demo.py`

### Simple MoveIt! planning
24 July 2021
Moveit functionality is on the moveit-programming branch. Check out this branch.

Open another terminal and enter the following

`rostopic pub -1 /ur_arm/moveit/goal_pose geometry_msgs/Pose "position:
  x: 1.2
  y: 0.0
  z: 1.5
orientation:
  x: 0.0
  y: 0.707
  z: 0.0
  w: 0.707"`
  
You should see the arm move to the planned position. You will probably see an error that the controllers failed. This is another gain tuning issue, as the arm is not quite getting to the goal state indicated.

### Controlling with Python
Once the gazebo simulation is running, execute

`python sdl_gazebo/python-nodes/demo.py`

To see the Kinect camera sensor information, run

`rosrun rviz rviz -d 'rospack sdl_robot_description'/cfg/point_cloud_config.rviz`

### Navigating the MIR 
```
python sdl_gazebo/python-nodes/mir_nav.py
```

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
