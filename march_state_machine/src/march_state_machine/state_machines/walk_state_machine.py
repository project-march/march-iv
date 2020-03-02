import smach

from .gait_state_machine import GaitStateMachine


class WalkStateMachine(GaitStateMachine):
    """A smach.StateMachine that implements a walking pattern gait."""

    def __init__(self, gait_name, is_transition_active=False):
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

        self.register_output_keys(['current_gait_name', 'transition_state_name', 'start_state'])

        self.default_start_state = 'right_open'

        self.open()
        self.add_subgait('right_open', succeeded='left_swing')
        self.add_subgait('right_swing', succeeded='left_swing', stopped='left_close')
        self.add_subgait('left_swing', succeeded='right_swing', stopped='right_close')
        self.add_subgait('right_close')
        self.add_subgait('left_close')
        self.close()

        if is_transition_active:
            self.register_input_keys(['start_state'])
            self.add_outcome('transition')

            self._states['right_swing'].add_outcome('transition')
            self._transitions['right_swing']['transition'] = 'transition'

            self._states['left_swing'].add_outcome('transition')
            self._transitions['left_swing']['transition'] = 'transition'

    def _update_once(self):
        """Set certain output variables in order to construct a possible transition."""
        self.userdata.current_gait_name = self._gait_name
        self.userdata.start_state = self._current_label

        self.userdata.transition_state_name = None

        if 'transition' in self._outcomes:
            self.userdata.transition_state_name = self._current_transitions['succeeded']

        return super(GaitStateMachine, self)._update_once()

    def execute(self, parent_ud=smach.UserData()):
        """Run this function on entry of this state, check if a start gait is passed to the state."""
        self._initial_state_label = self.default_start_state

        if 'transition' in self._outcomes:
            start_state = parent_ud.start_state

            if start_state is not None:
                self._initial_state_label = start_state
                parent_ud.start_state = None

        return super(WalkStateMachine, self).execute(parent_ud)

    def add_outcome(self, label):
        """Add new outcome to the state."""
        self._outcomes.add(label)
