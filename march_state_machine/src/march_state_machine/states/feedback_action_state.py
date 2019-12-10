import rospy
import smach_ros


class FeedbackActionState(smach_ros.SimpleActionState):
    def __init__(self, action_topic, action, goal, outcomes):
        super(FeedbackActionState, self).__init__(action_topic, action, goal,
                                                  outcomes=outcomes)
