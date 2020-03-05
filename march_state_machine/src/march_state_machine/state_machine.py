from multiprocessing.pool import ThreadPool

import rospy
import smach
import smach_ros

from . import launch_sm
from .healthy_sm import HealthyStateMachine
from .sounds import Sounds
from .states.empty_state import EmptyState
from .states.safety_state import SafetyState
from .states.shutdown_state import ShutdownState


def main():
    rospy.init_node('state_machine')

    sm = create_sm()
    rospy.on_shutdown(sm.request_preempt)
    sm.get_registered_outcomes()

    sis = None
    if rospy.get_param('~state_machine_viewer', False):
        rospy.loginfo('Starting state_machine_viewer')
        sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
        sis.start()

    pool = ThreadPool(processes=1)
    pool.apply_async(sm.execute)

    rospy.spin()

    pool.close()
    pool.join()
    if sis:
        sis.stop()


def create_userdata():
    """Creates userdata for all state machines.

    :rtype smach.Userdata()
    :returns created userdata
    """
    userdata = smach.UserData()
    if rospy.get_param('~sounds', False):
        userdata.sounds = Sounds()
    else:
        userdata.sounds = None
    return userdata


def create_sm():
    sm = smach.StateMachine(outcomes=['DONE'])
    sm.userdata = create_userdata()
    with sm:
        smach.StateMachine.add('LAUNCH', launch_sm.create(),
                               transitions={'succeeded': 'HEALTHY', 'preempted': 'SHUTDOWN', 'failed': 'SHUTDOWN'})

        safety_concurrence = smach.Concurrence(outcomes=['succeeded', 'failed', 'preempted'],
                                               default_outcome='failed',
                                               outcome_map={'failed': {'SAFETY': 'invalid',
                                                                       'STATE_MACHINE': 'preempted'},
                                                            'preempted': {'SAFETY': 'preempted',
                                                                          'STATE_MACHINE': 'preempted'},
                                                            'succeeded': {'SAFETY': 'valid',
                                                                          'STATE_MACHINE': 'succeeded'}},
                                               child_termination_cb=lambda _: True, input_keys=['sounds'],
                                               output_keys=['sounds'])

        with safety_concurrence:
            smach.Concurrence.add('SAFETY', SafetyState())
            smach.Concurrence.add('STATE_MACHINE', HealthyStateMachine())

        smach.StateMachine.add('HEALTHY', safety_concurrence,
                               transitions={'succeeded': 'SHUTDOWN', 'failed': 'ERROR', 'preempted': 'SHUTDOWN'})
        smach.StateMachine.add('ERROR', EmptyState(),
                               transitions={'succeeded': 'SHUTDOWN'})
        smach.StateMachine.add('SHUTDOWN', ShutdownState(), transitions={'succeeded': 'DONE'})

    return sm
