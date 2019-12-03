import threading

import rospy
import smach

from march_state_machine.control_flow import control_flow


class IdleState(smach.State):
    """State in which the exoskeleton is not moving.

    Listens to instructions from the input device and reacts if they are known transitions.
    """

    def __init__(self, outcomes=None):
        if outcomes is None:
            outcomes = []
        super(IdleState, self).__init__(outcomes=outcomes)

        self._result_gait = None
        self._trigger_event = threading.Event()

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self._trigger_event.clear()
        self._result_gait = None

        control_flow.reset_gait()
        control_flow.set_stopped_callback(self._stopped_cb)
        control_flow.set_gait_selected_callback(self._gait_cb)

        self._trigger_event.wait()
        control_flow.clear_callbacks()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self._result_gait:
            return self._result_gait

        return 'failed'

    def _stopped_cb(self):
        rospy.logwarn('Idle state does not respond to stop')
        control_flow.reset_stop()

    def _gait_cb(self, gait):
        if gait in self.get_registered_outcomes():
            rospy.loginfo('Accepted {0}'.format(gait))
            self._result_gait = gait
            control_flow.gait_accepted()
            self._trigger_event.set()
        else:
            rospy.logwarn('The {0} is not a possible gait in the current state'.format(gait))
            control_flow.gait_rejected()

    def request_preempt(self):
        super(IdleState, self).request_preempt()
        self._trigger_event.set()
