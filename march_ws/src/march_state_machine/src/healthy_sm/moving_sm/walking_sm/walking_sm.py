#!/usr/bin/env python
import smach
import rospy

from march_custom_msgs.srv import PerformGait

class MoveToPose(smach.State):
    def __init__(self, gait_name):
        self.gait_name = gait_name
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Trying to perform ' + self.gait_name)
        perform_gait_service = rospy.ServiceProxy('march/perform_gait', PerformGait)

        result = perform_gait_service(self.gait_name)
        rospy.loginfo(result)
        if result.success:
            return 'succeeded'
        else:
            return 'failed'


def create():
    sm_walking = smach.StateMachine(outcomes=['succeeded', 'failed'])
    # Open the container
    with sm_walking:
        # Add states to the container
        smach.StateMachine.add('RIGHT_STEP_OPEN', MoveToPose("right_step_open"),
                               transitions={'succeeded': 'LEFT_SWING',
                                            'failed': 'failed'})
        smach.StateMachine.add('LEFT_SWING', MoveToPose("left_swing"),
                               transitions={'succeeded': 'RIGHT_SWING',
                                            'failed': 'failed'})
        smach.StateMachine.add('RIGHT_SWING', MoveToPose("right_swing"),
                               transitions={'succeeded': 'LEFT_SWING',
                                            'failed': 'failed'})


    return sm_walking
