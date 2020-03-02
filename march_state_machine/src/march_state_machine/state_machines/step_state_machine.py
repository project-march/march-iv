import rospy
import smach
from std_msgs.msg import String

from march_state_machine.control_flow import control_flow
from march_state_machine.states.gait_state import GaitState


class StepStateMachine(smach.Sequence):
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
        super(StepStateMachine, self).__init__(outcomes=['succeeded', 'preempted', 'failed'],
                                               connector_outcome='succeeded')
        self._gait_name = gait_name

        if subgaits is None:
            subgaits = ['right_open', 'left_close']

        self.open()
        for subgait in subgaits:
            smach.Sequence.add(subgait, GaitState(gait_name, subgait), transitions={'aborted': 'failed'})
        self.close()

        self.register_termination_cb(self._termination_cb)
        self._gait_publisher = rospy.Publisher('/march/gait/current', String, queue_size=10)

    def execute(self, ud=smach.UserData()):
        self._gait_publisher.publish(self._gait_name)
        return super(StepStateMachine, self).execute(ud)

    @staticmethod
    def _termination_cb(_userdata, _terminal_states, _outcome):
        control_flow.gait_finished()
