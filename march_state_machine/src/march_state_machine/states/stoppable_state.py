import rospy

from feedback_action_state import FeedbackActionState
from march_shared_resources.msg import GaitNameAction, GaitNameGoal
from march_state_machine.control_flow import control_flow


class StoppableState(FeedbackActionState):
    def __init__(self, gait_name, subgait_name):
        self.subgait_name = subgait_name
        outcomes = ['succeeded', 'preempted', 'aborted', 'stopped']

        super(StoppableState, self).__init__('/march/gait/perform', GaitNameAction,
                                             GaitNameGoal(name=gait_name, subgait_name=self.subgait_name),
                                             outcomes=outcomes,
                                             input_keys=['stop_pressed'], output_keys=['stop_pressed'])

    def execute(self, userdata):
        result = super(StoppableState, self).execute(userdata)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if control_flow.stop_pressed():
            control_flow.reset_stop()
            control_flow.reset_gait()
            return 'stopped'

        while control_flow.is_paused() and not rospy.core.is_shutdown():
            rospy.loginfo_throttle(5, 'Gait is paused')

        return result
