import rospy
import smach
from std_msgs.msg import String

from march_state_machine.control_flow import control_flow
from march_state_machine.states.gait_state import GaitState
from march_state_machine.states.stoppable_state import StoppableState


class GaitStateMachine(smach.StateMachine):
    """A march gait implemented as a smach.StateMachine."""

    def __init__(self, gait_name):
        super(GaitStateMachine, self).__init__(outcomes=['succeeded', 'preempted', 'failed'], input_keys=['sounds'],
                                               output_keys=['sounds'])
        self._gait_name = gait_name
        self._gait_publisher = rospy.Publisher('/march/gait/current', String, queue_size=10)

        self.register_start_cb(self._start_cb)
        self.register_termination_cb(self._termination_cb)

    def add_subgait(self, subgait_name, succeeded='succeeded', stopped=None):
        """Adds a subgait state to the current gait state machine.

        The state machine container needs to be opened before adding all subgaits.
        Otherwise it will throw an smach.InvalidConstructionError.

        :Example:
            sm_walk = GaitStateMachine('walk')
            with sm_walk:
                sm_walk.add_subgait('right_open', succeeded='left_swing')
                sm_walk.add_subgait('right_swing', succeeded='left_swing', stopped='left_close')
                sm_walk.add_subgait('left_swing', succeeded='right_swing', stopped='right_close')
                sm_walk.add_subgait('right_close')
                sm_walk.add_subgait('left_close')

        :type subgait_name: str
        :param subgait_name: Subgait name of the current gait
        :type succeeded: str
        :param succeeded: State to transition to when the subgait is successful
        :type stopped: str
        :param stopped: Name of state to transition to when the subgait has been stopped.
                        When this parameter is given it implies that the state to be added
                        can be stopped.
        :raises smach.InvalidConstructionError: when the container was not opened
        """
        self.assert_opened()
        if stopped:
            smach.StateMachine.add(subgait_name, StoppableState(self._gait_name, subgait_name),
                                   transitions={'succeeded': succeeded, 'stopped': stopped, 'aborted': 'failed'})
        else:
            smach.StateMachine.add(subgait_name, GaitState(self._gait_name, subgait_name),
                                   transitions={'succeeded': succeeded, 'aborted': 'failed'})

    def execute(self, ud=smach.UserData()):
        self._gait_publisher.publish(self._gait_name)
        return super(GaitStateMachine, self).execute(ud)

    @staticmethod
    def _start_cb(userdata, _initial_states):
        if userdata.sounds:
            userdata.sounds.play('gait_start')

    @staticmethod
    def _termination_cb(userdata, _terminal_states, _outcome):
        if userdata.sounds:
            userdata.sounds.play('gait_end')
        control_flow.gait_finished()
