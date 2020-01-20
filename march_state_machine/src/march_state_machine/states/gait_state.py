from feedback_action_state import FeedbackActionState

from march_shared_resources.msg import GaitNameAction, GaitNameGoal


class GaitState(FeedbackActionState):
    def __init__(self, gait_name, subgait_name, outcomes=None):
        if outcomes is None:
            outcomes = ['succeeded', 'preempted', 'aborted']
        super(GaitState, self).__init__('/march/gait/perform',
                                        GaitNameAction,
                                        GaitNameGoal(name=gait_name, subgait_name=subgait_name),
                                        outcomes=outcomes)
