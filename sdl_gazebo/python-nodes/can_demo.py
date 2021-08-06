from geometry_msgs.msg import PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray
from rospy import Subscriber, Publisher
import rospy

locked = False

def main():
    # topic from mir library to publish navigation goal to
    pose_pub = Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    status_sub = Subscriber("move_base/status", GoalStatusArray, lock_control)
    arm_pub = Publisher("/ur_arm/moveit/goal_pose", Pose, queue_size=1)

    pose = PoseStamped()
    pose.header.seq = 0
    pose.header.frame_id = "map"
    pose.pose.position.x = -2.748
    pose.pose.position.y = -1.767

    pose.pose.orientation.z = 1
    pose.pose.orientation.w = -0.00900788442947

    # Publishers need some time to get started
    rospy.sleep(1)
    pose_pub.publish(pose)
    rospy.sleep(1)

    # While MIR is still navigating to a goal, don't publish next goal
    while locked:
        pass
    
    pose = Pose()
    pose.position.x = 1.35
    pose.position.z = .9
    pose.orientation.y = .5
    pose.orientation.w = .5
    pose.orientation.x = 0.5
    pose.orientation.z = 0.5
    arm_pub.publish(pose)

def lock_control(data):
    global locked
    if not data.status_list or data.status_list[0].text == "Goal reached.":
        locked = False
    else:
        locked = True


if __name__ == "__main__":
    rospy.init_node("mir_nav")
    main()
