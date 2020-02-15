import smach_ros


class FeedbackActionState(smach_ros.SimpleActionState):
    def __init__(self, action_topic, action, goal, outcomes, input_keys=None):
        if input_keys is None:
            input_keys = []
        super(FeedbackActionState, self).__init__(action_topic, action, goal,
                                                  outcomes=outcomes, input_keys=input_keys)
