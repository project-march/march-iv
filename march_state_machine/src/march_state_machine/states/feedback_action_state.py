import smach_ros


class FeedbackActionState(smach_ros.SimpleActionState):
    def __init__(self, action_topic, action, goal, outcomes, input_keys=None, output_keys=None):
        if input_keys is None:
            input_keys = []
        if output_keys is None:
            output_keys = []

        super(FeedbackActionState, self).__init__(action_topic, action, goal,
                                                  outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
