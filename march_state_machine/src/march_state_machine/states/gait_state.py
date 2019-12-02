from feedback_action_state import FeedbackActionState

from march_shared_resources.msg import GaitNameAction, GaitNameGoal


class GaitState(FeedbackActionState):
    def __init__(self, gait_name, subgait_name, outcomes=None, input_keys=None, output_keys=None):
        if outcomes is None:
            outcomes = ['succeeded', 'preempted', 'aborted']
        if output_keys is None:
            output_keys = []
        if input_keys is None:
            input_keys = []
        super(GaitState, self).__init__('/march/gait/perform',
                                        GaitNameAction,
                                        GaitNameGoal(name=gait_name, subgait_name=subgait_name),
                                        outcomes=outcomes,
                                        input_keys=input_keys,
                                        output_keys=output_keys)
