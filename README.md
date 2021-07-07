# sdl-packages

This repository holds code for the SDL simulation project at Argonne National Laboratory/University of Chicago

## Setup
Assuming ROS Melodic is installed.

Add ROS descriptions for the robots we are using

`sudo apt-get install ros-melodic-universal-robot`
`sudo apt install ros-noetic-mir-robot`

Create a workspace for the project

`mkdir -p catkin_ws/src`

Go into the workspace and clone the repository
`cd catkin_ws/src`
`git clone https://github.com/dsquez/sdl-packages.git`

Go back to src and build
`cd catkin_ws`
`catkin_make`

Source the new workspace
`source catkin_ws/devel/setup.bash`

## Running
To see the rviz visualization, make sure ros and the workspace are sourced and run
`roslaunch sdl_robot_description view_sdl.launch`

To launch the gazebo simulaion, run
`roslaunch sdl_gazebo sdl_robot.launch`

Moveit is not currently working (7 July 2021)
