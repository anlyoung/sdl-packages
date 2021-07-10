from rospy import Subscriber, Publisher
from sensor_msgs.msg import PointCloud2
from time import sleep
import rospy
from geometry_msgs.msg import Twist

class MirRobot:

    def run(self):
        rospy.init_node("python_sub")

        self.depth_sub = Subscriber("/camera/depth/points", PointCloud2, self.depth_callback)

        self.wheel_pub = Publisher("/wheel_controller/cmd_vel", Twist, queue_size=1)
        while not rospy.is_shutdown():
            self.move_forward()

    def depth_callback(self, params):
        print(params)

    def navigate(self):
        return

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 10
        msg.linear.y = 10
        msg.linear.z = 10
        self.wheel_pub.publish(msg)

if __name__ == "__main__":
    robot = MirRobot()
    robot.run()
