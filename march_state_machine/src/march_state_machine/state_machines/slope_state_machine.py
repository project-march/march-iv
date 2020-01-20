from .gait_state_machine import GaitStateMachine


class SlopeStateMachine(GaitStateMachine):
    """A smach.StateMachine that implements a walking pattern gait for a slope."""

    def __init__(self, gait_name):
        """Initializes the slope pattern gait.

        The gait will be initialized with the following subgaits:
        * right_open
        * left_open
        * right_swing
        * left_swing
        * right_close
        * left_close
        Where right_swing and left_swing can be stopped and will transition
        to either right_close or left_close.

        :type gait_name: str
        :param gait_name: Name of the slope pattern gait
        """
        super(SlopeStateMachine, self).__init__(gait_name)

        self.open()
        self.add_subgait('right_open', succeeded='left_open')
        self.add_subgait('left_open', succeeded='right_swing')
        self.add_subgait('right_swing', succeeded='left_swing', stopped='left_close')
        self.add_subgait('left_swing', succeeded='right_swing', stopped='right_close')
        self.add_subgait('left_close')
        self.add_subgait('right_close')
        self.close()
