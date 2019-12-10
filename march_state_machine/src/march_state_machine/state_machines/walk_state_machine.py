from .gait_state_machine import GaitStateMachine


class WalkStateMachine(GaitStateMachine):

    def __init__(self, gait_name):
        super(WalkStateMachine, self).__init__(gait_name)

        self.open()
        self.add_subgait('right_open', succeeded='left_swing')
        self.add_subgait('right_swing', succeeded='left_swing', stopped='left_close')
        self.add_subgait('left_swing', succeeded='right_swing', stopped='right_close')
        self.add_subgait('right_close')
        self.add_subgait('left_close')
        self.close()
