from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from rospy import Subscriber, Publisher
import rospy

locked = False

def main():
    # topic from mir library to publish navigation goal to
    pose_pub = Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    status_sub = Subscriber("move_base/status", GoalStatusArray, lock_control)

    pose = PoseStamped()
    pose.header.seq = 0
    pose.header.frame_id = "map"
    pose.pose.position.x = 4
    pose.pose.position.y = 2

    pose.pose.orientation.z = 0.923879533652
    pose.pose.orientation.w = 0.382683429611

    # Publishers need some time to get started
    rospy.sleep(1)
    pose_pub.publish(pose)
    rospy.sleep(1)

    # While MIR is still navigating to a goal, don't publish next goal
    while locked:
        pass

    pose.pose.position.x = -6
    pose.pose.position.y = -3
    pose_pub.publish(pose)

def lock_control(data):
    global locked
    if data.status_list:
        locked = True
    else:
        locked = False


if __name__ == "__main__":
    rospy.init_node("mir_nav")
    main()
