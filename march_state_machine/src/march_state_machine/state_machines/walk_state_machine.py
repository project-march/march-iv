import smach

from march_state_machine.states.stoppable_state import StoppableState

from .gait_state_machine import GaitStateMachine


class WalkStateMachine(GaitStateMachine):
    """A smach.StateMachine that implements a walking pattern gait."""

    def __init__(self, gait_name, transition_chain=None):
        """Initializes the walking pattern gait.

        The gait will be initialized with the following subgaits:
        * right_open
        * right_swing
        * left_swing
        * right_close
        * left_close
        Where right_swing and left_swing can be stopped and will transition
        to either right_close or left_close.

        :type gait_name: str
        :param gait_name: Name of the walking pattern gait
        """
        super(WalkStateMachine, self).__init__(gait_name)

        if transition_chain:
            self._outcomes = ['succeeded', 'preempted', 'failed', 'transition']

        self._transition_chain = transition_chain
        self.register_output_keys(['current_gait_name', 'new_gait_name', 'transition_state_name', 'next_state_name'])
        self.register_input_keys(['start_state'])

        self.default_start_state = 'right_open'

        self.open()
        self.add_subgait('right_open', succeeded='left_swing')
        self.add_subgait('right_swing', succeeded='left_swing', stopped='left_close')
        self.add_subgait('left_swing', succeeded='right_swing', stopped='right_close')
        self.add_subgait('right_close')
        self.add_subgait('left_close')
        self.close()

    def add_subgait(self, subgait_name, succeeded='succeeded', stopped=None):
        """Override method to check whether the gait is part of a transition chain."""
        if self._transition_chain and stopped:
            smach.StateMachine.add(subgait_name,
                                   StoppableState(self._gait_name, subgait_name, self._transition_chain),
                                   transitions={'succeeded': succeeded, 'stopped': stopped, 'aborted': 'failed',
                                                'transition': 'transition'})
        else:
            super(WalkStateMachine, self).add_subgait(subgait_name, succeeded, stopped)

    def _update_once(self):
        """Set certain output variables in order to construct a possible transition."""
        self.userdata.current_gait_name = self._gait_name
        self.userdata.next_state_name = self._current_label

        self.userdata.transition_state_name = None
        self.userdata.new_gait_name = None

        if self._transition_chain:
            self.userdata.transition_state_name = self._current_transitions['succeeded']

        return super(GaitStateMachine, self)._update_once()

    def execute(self, parent_ud=smach.UserData()):
        """Run this function on entry of this state, check if a start gait is passed to the state."""
        start_state = parent_ud.start_state

        if start_state is not None:
            self._initial_state_label = start_state
            parent_ud.next_state_name = None
        else:
            self._initial_state_label = self.default_start_state

        return super(WalkStateMachine, self).execute(parent_ud)
