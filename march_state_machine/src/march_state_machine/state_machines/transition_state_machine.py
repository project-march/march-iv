import ast

import rospy
import smach

from march_shared_resources.srv import Trigger
from march_state_machine.states.transition_state import TransitionState


class StateMachineWithTransition(smach.StateMachine):
    def __init__(self, transition_sequence, outcomes=None):
        self._default_outcomes = ['succeeded', 'preempted', 'failed']

        if outcomes is not None:
            self._default_outcomes = outcomes

        super(StateMachineWithTransition, self).__init__(outcomes=self._default_outcomes)

        with self.opened():
            smach.StateMachine.add('transition', TransitionState(transition_sequence),
                                   transitions={'aborted': 'failed'})

    def add(self, label, state, transitions=None, remapping=None, default_start=False):
        """Override the add-function to set the necessary values in the transition state within this state machine."""
        if transitions is None:
            transitions = {'succeeded': 'succeeded', 'failed': 'failed', 'transition': 'transition'}

        self._states['transition'].add_outcome(label)
        self._transitions['transition'][label] = label

        if default_start:
            self._initial_state_label = label

        self.open()
        result = super(StateMachineWithTransition, self).add(label, state, transitions, remapping)
        self.close()

        return result

    def execute(self, parent_ud=smach.UserData()):
        """Reset the initial start state."""
        self.userdata.start_state = None

        try:
            rospy.wait_for_service('/march/gait_selection/get_version_map', 1)
            get_gait_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)

            gait_version_map = ast.literal_eval(get_gait_version_map().message)

            for label in self._states.copy():
                if label not in gait_version_map and label != 'transition':
                    rospy.logwarn('State {lb} does not have a valid gait file, '
                                  'removing state from transition state machine'.format(lb=label))

                    self._states.pop(label)
                    self._states['transition'].remove_outcome(label)
                    self._transitions['transition'].pop(label)

            return super(StateMachineWithTransition, self).execute(parent_ud)

        except rospy.ROSException and ValueError:
            rospy.logwarn('Transition state could not verify the transitions, do not use rocker-switch buttons!')
            return super(StateMachineWithTransition, self).execute(parent_ud)
