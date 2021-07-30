import rospy
import smach

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["waiting_table2"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"

class WaitingForDropOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["drop_off2_complete"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"

class InProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["process_done"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"
