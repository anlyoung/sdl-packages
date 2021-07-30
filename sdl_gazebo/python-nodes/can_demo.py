from geometry_msgs.msg import PoseStamped, Pose, Point
from actionlib_msgs.msg import GoalStatusArray
from rospy import Subscriber, Publisher
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import time

locked = False

def main():
    global pose_pub
    global status_sub
    global arm_pub
    # topic from mir library to publish navigation goal to
    pose_pub = Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    arm_pub = Publisher("/ur_arm/moveit/goal_pose", Pose, queue_size=1)
    arm_status_sub = Subscriber("/move_group/status", GoalStatusArray, arm_status)

    pose = PoseStamped()
    pose.header.seq = 0
    pose.header.frame_id = "map"
    pose.pose.position.x = -2.9
    pose.pose.position.y = -2

    pose.pose.orientation.z = 1
    pose.pose.orientation.w = -0.00900788442947

    # Publishers need some time to get started
    rospy.sleep(1)
    fold_arm()
    pose_pub.publish(pose)
    rospy.sleep(1)
    marker_sub = Subscriber("/ar_pose_marker", AlvarMarkers, marker_position)
    status_sub = Subscriber("move_base/status", GoalStatusArray, lock_control)
    rospy.spin()
    # While MIR is still navigating to a goal, don't publish next goal
    # while locked:
    #     pass
    
    # pose = Pose()
    # pose.position.x = 1.24
    # pose.position.z = .787
    # pose.orientation.y = .707
    # pose.orientation.w = .707
    # arm_pub.publish(pose)

def lock_control(data):
    global locked
    if not data.status_list or data.status_list[0].text == "Goal reached.":
        locked = False
    else:
        locked = True

#origin of Kinect sensor at x = .6, y = 0, z = 1.4 in arm coordinates
def kinect_to_arm(position):
    out = Point()
    out.x = .6
    out.z = 1.4
    out.z += position.y
    out.y += position.x
    out.x += position.z
    return out

done = False

def marker_position(data):
    global locked
    global done
    if not locked and data.markers and not done:
        pose = Pose()
        pose.position = kinect_to_arm(data.markers[0].pose.pose.position)
        pose.orientation.y = .707
        pose.orientation.w = .707
        global arm_pub
        arm_pub.publish(pose)
        done = True

def arm_status(data):
    pass

def fold_arm():
    global arm_pub
    pose = Pose()
    pose.position.x = 0.
    pose.position.z = 2.
    arm_pub.publish(pose)
    rospy.sleep(5)

if __name__ == "__main__":
    rospy.init_node("mir_nav")
    main()
