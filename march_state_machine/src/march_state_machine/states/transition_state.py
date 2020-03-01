import rospy

from march_shared_resources.msg import GaitNameAction, GaitNameGoal
from march_state_machine.control_flow import control_flow
from march_state_machine.states.feedback_action_state import FeedbackActionState


class TransitionState(FeedbackActionState):
    def __init__(self, transition_sequence, outcomes=None, input_keys=None):
        self._transition_sequence = transition_sequence

        if outcomes is None:
            outcomes = ['succeeded', 'preempted', 'aborted']

        if input_keys is None:
            input_keys = ['current_gait_name', 'transition_state_name', 'start_state']

        super(TransitionState, self).__init__('/march/gait/perform', GaitNameAction, None,
                                              outcomes=outcomes, input_keys=input_keys)

    def execute(self, ud):
        """Execute the transition step using the given input keys."""
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        current_gait_name = ud.current_gait_name
        transition_subgait_name = ud.transition_state_name

        new_gait_name = self.get_transition_gait(current_gait_name)

        if new_gait_name is None:
            rospy.logwarn('Transition from {current} to {transition}'.format(current=current_gait_name,
                                                                             transition=new_gait_name))
            self._goal = GaitNameGoal(name=current_gait_name, subgait_name=transition_subgait_name)
        else:
            rospy.logdebug('Current gait name: {cgn}, new gait name: {ngn}, transition subgait name: {sgn}'
                           .format(cgn=current_gait_name, ngn=new_gait_name, sgn=transition_subgait_name))

            self._goal = GaitNameGoal(name=new_gait_name, old_name=current_gait_name,
                                      subgait_name=transition_subgait_name)

        result = super(TransitionState, self).execute(ud)

        if result == 'succeeded':
            if new_gait_name:
                return new_gait_name
            return current_gait_name

        return result

    def add_outcome(self, label):
        """Add extra outcome to the predefined set of outcomes."""
        self._outcomes.add(label)

    def remove_outcome(self, label):
        """Remove outcome of the predefined set of outcomes and transition sequence."""
        try:
            self._outcomes.remove(label)
            self._transition_sequence.remove(label)
        except ValueError:
            pass

    def get_transition_gait(self, gait_name):
        """Get the requested transition of the passed transition chain."""
        if gait_name not in self._transition_sequence:
            return None

        transition_direction = control_flow.get_transition_integer()

        if transition_direction:
            control_flow.reset_transition()

            if transition_direction == 1:
                if self._transition_sequence[-1] == gait_name:
                    return None

            if transition_direction == -1:
                if self._transition_sequence[0] == gait_name:
                    return None

            transition_index = self._transition_sequence.index(gait_name) + transition_direction
            return self._transition_sequence[transition_index]

        return None
