import smach

from march_state_machine.states.gait_state import GaitState

from .gait_state_machine import GaitStateMachine


class StepStateMachine(GaitStateMachine):
    """A smach.StateMachine that implements a gait with a sequence of subgaits."""

    def __init__(self, gait_name, subgaits=None):
        """Initializes the state machine with a gait name and a sequence of subgaits.

        The subgaits will be performed in the sequence given.

        :type gait_name: str
        :param gait_name: Name of the gait to perform in sequence
        :type subgaits: list(str)
        :param subgaits: List of subgaits to be performed in the given order.
                         When None, the subgaits will be ['right_open', 'left_close'].
        """
        super(StepStateMachine, self).__init__(gait_name)

        if subgaits is None:
            subgaits = ['right_open', 'left_close']

        self.open()
        for subgait in subgaits:
            smach.Sequence.add_auto(subgait, GaitState(gait_name, subgait), connector_outcomes=['succeeded'],
                                    transitions={'aborted': 'failed'})
        self.close()
