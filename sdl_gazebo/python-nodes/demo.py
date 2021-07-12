from __future__ import print_function
from rospy import Subscriber, Publisher
from sensor_msgs.msg import Image
from time import sleep
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import math


class MirRobot:

    def run(self):
        rospy.init_node("python_sub")

        self.action = "forward"

        self.depth_sub = Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.wheel_pub = Publisher("/wheel_controller/cmd_vel", Twist, queue_size=1)

        self.navigate()

    def depth_callback(self, params):

        from cv_bridge import CvBridge
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(params, "32FC1")

        for i in range(len(image)):
            for j in range(len(image[i])):
                if not math.isnan(image[i][j]) and i == 200:
                    print("turn")
                    self.action = "turn"
                    return
        self.action = "forward"

    def turn(self):
        msg = Twist()
        msg.angular.x = .5
        msg.angular.y = .5
        msg.angular.z = .5
        self.wheel_pub.publish(msg)

    def navigate(self):
        while not rospy.is_shutdown():
            if self.action == "forward":
                self.move_forward()
            elif self.action == "turn":
                self.turn()

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 10
        self.wheel_pub.publish(msg)

if __name__ == "__main__":
    robot = MirRobot()
    robot.run()
