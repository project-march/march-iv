import smach

from march_state_machine.control_flow import control_flow
from march_state_machine.states.gait_state import GaitState


class StepStateMachine(smach.Sequence):

    def __init__(self, gait_name, subgaits=None):
        super(StepStateMachine, self).__init__(outcomes=['succeeded', 'preempted', 'failed'],
                                               connector_outcome='succeeded')

        if subgaits is None:
            subgaits = ['right_open', 'left_close']

        self.open()
        for subgait in subgaits:
            smach.Sequence.add(subgait, GaitState(gait_name, subgait), transitions={'aborted': 'failed'})
        self.close()

        self.register_termination_cb(self._termination_cb)

    @staticmethod
    def _termination_cb(_userdata, _terminal_states, _outcome):
        control_flow.gait_finished()
