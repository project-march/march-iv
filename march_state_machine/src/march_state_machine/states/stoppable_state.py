from feedback_action_state import FeedbackActionState
import rospy

from march_shared_resources.msg import GaitNameAction, GaitNameGoal
from march_state_machine.control_flow import control_flow


class StoppableState(FeedbackActionState):
    def __init__(self, gait_name, subgait_name, transition_chain=None):
        self._gait_name = gait_name
        self._subgait_name = subgait_name
        self._transition_chain = transition_chain

        super(StoppableState, self).__init__('/march/gait/perform', GaitNameAction,
                                             GaitNameGoal(name=gait_name, subgait_name=self._subgait_name),
                                             outcomes=['succeeded', 'preempted', 'aborted', 'stopped'])
        if self._transition_chain:
            self._outcomes = ['succeeded', 'preempted', 'aborted', 'stopped', 'transition']
            self.register_output_keys(['current_gait_name', 'new_gait_name', 'next_subgait'])

    def execute(self, ud):
        """Run this function on entry of this state."""
        result = super(StoppableState, self).execute(ud)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if control_flow.stop_pressed():
            control_flow.reset_stop()
            control_flow.reset_gait()
            return 'stopped'

        if control_flow.is_transition():
            if self.check_transition():
                ud.new_gait_name = self.get_transition()
                control_flow.reset_transition()
                return 'transition'
            else:
                control_flow.reset_transition()
                rospy.logwarn('Not a valid transition requested from the current gait')

        while control_flow.is_paused() and not rospy.core.is_shutdown():
            rospy.loginfo_throttle(5, 'Gait is paused')

        return result

    def check_transition(self):
        """Check if a transition is allowed and possible of the current state."""
        if self._transition_chain is None:
            return False

        if 'transition' not in self._outcomes:
            return False

        transition_direction = control_flow.get_transition_integer()

        if self._gait_name not in self._transition_chain:
            return False

        if transition_direction == 1:
            if self._transition_chain[-1] == self._gait_name:
                return False

        if transition_direction == -1:
            if self._transition_chain[0] == self._gait_name:
                return False

        return True

    def get_transition(self):
        """Get the requested transition of the passed transition chain."""
        transition_direction = control_flow.get_transition_integer()
        transition_index = self._transition_chain.index(self._gait_name) + transition_direction
        return self._transition_chain[transition_index]
