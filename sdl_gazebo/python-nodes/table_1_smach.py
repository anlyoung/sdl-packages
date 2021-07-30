import rospy
import smach

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["table1_process_start"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"

class InProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["process_done"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"

class WaitingForPickup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["waiting_before_table1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"
