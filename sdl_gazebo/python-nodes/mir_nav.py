from geometry_msgs.msg import PoseStamped
from rospy import Subscriber, Publisher
import rospy

def main():
    # topic from mir library to publish navigation goal to
    pose_pub = Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    pose = PoseStamped()
    pose.header.seq = 1
    pose.header.frame_id = "map"
    pose.header.stamp.secs = 1626918721
    pose.header.stamp.nsecs = 445948072
    pose.pose.position.x = 0.083110332489
    pose.pose.position.y = -0.663573682308

    pose.pose.orientation.z = 0.923879533652
    pose.pose.orientation.w = 0.382683429611

    pose_pub.publish(pose)
    rospy.sleep(10)


    pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("mir_nav")
    main()
