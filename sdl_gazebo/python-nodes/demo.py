from rospy import Subscriber
from sensor_msgs.msg import PointCloud2
from time import sleep
import rospy


def main():
    rospy.init_node("python_sub")
    depth_sub = Subscriber("/camera/depth/points", PointCloud2, depth_callback)
    rospy.spin()

def depth_callback(params):
    print(params)

if __name__ == "__main__":
    main()
