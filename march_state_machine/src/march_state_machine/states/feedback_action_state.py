import smach_ros
import rospy


class FeedbackActionState(smach_ros.SimpleActionState):
    def __init__(self, action_topic, action, goal, outcomes, input_keys=None, output_keys=None):
        if output_keys is None:
            output_keys = []
        if input_keys is None:
            input_keys = []
        super(FeedbackActionState, self).__init__(action_topic, action, goal,
                                                  preempt_timeout=rospy.Duration(0),
                                                  outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
