import smach
import rospy
from march_shared_resources.msg import GaitInstruction
from march_state_machine.ControlFlow import control_flow


class IdleState(smach.State):
    """State in which the exoskeleton is not moving.
    Listens to instructions from the input device and reacts if they are known transitions.
    """
    def __init__(self, outcomes=[]):
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, userdata):
        # Sleep for a cycle, Simulate rospy.spinOnce() which is not available.
        # We don't want to spin because then we can't exit on a gait instruction.
        rate = rospy.Rate(100)
        # Remove old gait instructions set in previous states.
        control_flow.reset_gait()
        while not rospy.core.is_shutdown():
            if self.preempt_requested():
                rospy.logwarn("preempted")
                self.service_preempt()
                return 'preempted'
            if control_flow.stop_pressed():
                rospy.logwarn("Idle state doesn't respond to stop")
                control_flow.reset_stop()
            if control_flow.gait_selected():
                result_gait = control_flow.gait_name()
                if result_gait in self.get_registered_outcomes():
                    control_flow.gait_accepted()
                    return result_gait
                else:
                    rospy.logwarn('The ' + result_gait + ' is not a possible gait in the current state')
                    control_flow.gait_rejected()
            rate.sleep()
