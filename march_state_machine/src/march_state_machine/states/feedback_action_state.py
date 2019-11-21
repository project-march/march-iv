import smach_ros
import rospy


class FeedbackActionState(smach_ros.SimpleActionState):
    def __init__(self, action_topic, action, goal, outcomes, input_keys=[], output_keys=[]):
        smach_ros.SimpleActionState.__init__(self, action_topic, action, goal,
                                             preempt_timeout=rospy.Duration(0),
                                             outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
