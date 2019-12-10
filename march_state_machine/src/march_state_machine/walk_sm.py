from .state_machines.gait_state_machine import GaitStateMachine


def create():
    sm_walk = GaitStateMachine('walk')
    with sm_walk:
        sm_walk.add_subgait('right_open', succeeded='left_swing')
        sm_walk.add_subgait('right_swing', succeeded='left_swing', stopped='left_close')
        sm_walk.add_subgait('left_swing', succeeded='right_swing', stopped='right_close')
        sm_walk.add_subgait('right_close')
        sm_walk.add_subgait('left_close')

    return sm_walk
