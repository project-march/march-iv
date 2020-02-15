import rospy

from march_shared_resources.msg import GaitNameAction, GaitNameGoal
from march_state_machine.states.feedback_action_state import FeedbackActionState


class TransitionState(FeedbackActionState):
    def __init__(self, outcomes=None, input_keys=None):
        if outcomes is None:
            outcomes = ['succeeded', 'preempted', 'aborted']
        if input_keys is None:
            input_keys = []

        super(TransitionState, self).__init__('/march/gait/perform', GaitNameAction, None,
                                              outcomes=outcomes, input_keys=input_keys)

    def execute(self, ud):
        """Check if a transition is allowed and possible of the current state."""
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        current_gait_name = ud.current_gait_name
        new_gait_name = ud.new_gait_name
        transition_subgait_name = ud.transition_state_name

        rospy.logdebug('Current gait name: {cgn}, new gait name: {ngn}, transition subgait name: {sgn}'
                       .format(cgn=current_gait_name, ngn=new_gait_name, sgn=transition_subgait_name))

        self._goal = GaitNameGoal(name=new_gait_name, old_name=current_gait_name, subgait_name=transition_subgait_name)

        return super(TransitionState, self).execute(ud)
