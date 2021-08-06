#!/usr/bin/env python

import rospy
from rospy.rostime import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sdl_msgs.msg import JointCommand
from std_msgs.msg import Header

def commandCallback(msg):
    print("got a message!")
    names = msg.names
    command = msg.command

    trajMsg = JointTrajectory()

    trajMsg.header = Header()
    trajMsg.header.stamp = rospy.Time.now()

    trajMsg.joint_names = names

    tempPoints = JointTrajectoryPoint()
    tempPoints.positions = command
    tempPoints.time_from_start = Duration(5.0)

    trajMsg.points = [tempPoints]
    

    pubber.publish(trajMsg)    
    
    

if __name__ == '__main__':
    rospy.init_node('controller_handoff_node')
    pubber = rospy.Publisher('/effort_traj_controller/command',JointTrajectory,queue_size=10)
    rate = rospy.Rate(50)
    rospy.Subscriber('/robot/limb/arm/joint_command', JointCommand, commandCallback)
    while not rospy.is_shutdown():
        rospy.spin()
    