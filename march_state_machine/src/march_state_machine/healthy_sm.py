import rospy
import smach

from march_state_machine import walk_sm
from march_state_machine import sit_sm
from march_state_machine import stand_sm
from march_state_machine import single_step_small_sm
from march_state_machine import single_step_normal_sm
from march_state_machine import side_step_left_sm
from march_state_machine import side_step_right_sm
from march_state_machine import side_step_left_small_sm
from march_state_machine import side_step_right_small_sm
from march_state_machine import sofa_sit_sm
from march_state_machine import sofa_stand_sm
from march_state_machine import tilted_path_sm
from march_state_machine import walk_small_sm
from march_state_machine import rough_terrain_high_step_sm
from march_state_machine import rough_terrain_middle_steps_sm
from march_state_machine import ramp_door_slope_up_sm
from march_state_machine import stairs_sm
from march_state_machine.states.idle_state import IdleState
from march_state_machine.states.gait_state import GaitState
from std_srvs.srv import Empty, EmptyRequest


class HealthyStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        if rospy.get_param('~unpause', False):
            unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause.wait_for_service()
            unpause(EmptyRequest())
        rospy.loginfo('March is fully operational')
        return 'succeeded'


def create():
    sm_healthy = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    # Open the container
    with sm_healthy:
        # Add states to the container
        smach.StateMachine.add('START', HealthyStart(),
                               transitions={'succeeded': 'UNKNOWN'})
        smach.StateMachine.add('UNKNOWN', IdleState(outcomes=['home_sit', 'home_stand', 'failed', 'preempted']),
                               transitions={
                                   'home_sit': 'HOME SIT',
                                   'home_stand': 'HOME STAND',
                               })

        # Movement states
        smach.StateMachine.add('HOME SIT', GaitState('home', 'home_sit'),
                               transitions={'succeeded': 'SITTING', 'aborted': 'UNKNOWN'})

        smach.StateMachine.add('HOME STAND', GaitState('home', 'home_stand'),
                               transitions={'succeeded': 'STANDING', 'aborted': 'UNKNOWN'})

        smach.StateMachine.add('GAIT WALK', walk_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIT', sit_sm.create(),
                               transitions={'succeeded': 'SITTING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT STAND', stand_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SINGLE STEP SMALL', single_step_small_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SINGLE STEP NORMAL', single_step_normal_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP LEFT', side_step_left_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP RIGHT', side_step_right_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP LEFT SMALL', side_step_left_small_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP RIGHT SMALL', side_step_right_small_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SOFA SIT', sofa_sit_sm.create(),
                               transitions={'succeeded': 'SOFA SITTING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SOFA STAND', sofa_stand_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT STAIRS UP', stairs_sm.create('stairs_up'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT STAIRS DOWN', stairs_sm.create('stairs_down'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT TILTED PATH', tilted_path_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT WALK SMALL', walk_small_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT RT HIGH STEP', rough_terrain_high_step_sm.create(),  # RT stands for Rough Terrain
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT RT MIDDLE STEPS', rough_terrain_middle_steps_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT RD SLOPE UP', ramp_door_slope_up_sm.create(),  # RD stands for Ramp and Door
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        # Idle states
        smach.StateMachine.add('SITTING', IdleState(outcomes=['gait_stand', 'preempted']),
                               transitions={'gait_stand': 'GAIT STAND'})
        smach.StateMachine.add('SOFA SITTING', IdleState(outcomes=['gait_sofa_stand', 'preempted']),
                               transitions={'gait_sofa_stand': 'GAIT SOFA STAND'})
        smach.StateMachine.add('STANDING', IdleState(outcomes=['gait_sit', 'gait_walk', 'gait_single_step_small',
                                                               'gait_single_step_normal', 'gait_side_step_left',
                                                               'gait_side_step_right', 'gait_side_step_left_small',
                                                               'gait_side_step_right_small', 'gait_sofa_sit',
                                                               'gait_stairs_up', 'gait_stairs_down',
                                                               'gait_set_ankle_from_2_5_to_min5',
                                                               'gait_walk_small', 'gait_rough_terrain_high_step',
                                                               'gait_rough_terrain_middle_steps',
                                                               'gait_ramp_door_slope_up',
                                                               'preempted']),
                               transitions={'gait_sit': 'GAIT SIT', 'gait_walk': 'GAIT WALK',
                                            'gait_single_step_small': 'GAIT SINGLE STEP SMALL',
                                            'gait_single_step_normal': 'GAIT SINGLE STEP NORMAL',
                                            'gait_side_step_left': 'GAIT SIDE STEP LEFT',
                                            'gait_side_step_right': 'GAIT SIDE STEP RIGHT',
                                            'gait_side_step_left_small': 'GAIT SIDE STEP LEFT SMALL',
                                            'gait_side_step_right_small': 'GAIT SIDE STEP RIGHT SMALL',
                                            'gait_sofa_sit': 'GAIT SOFA SIT',
                                            'gait_stairs_up': 'GAIT STAIRS UP',
                                            'gait_stairs_down': 'GAIT STAIRS DOWN',
                                            'gait_set_ankle_from_2_5_to_min5': 'GAIT TILTED PATH',
                                            'gait_walk_small': 'GAIT WALK SMALL',
                                            'gait_rough_terrain_high_step': 'GAIT RT HIGH STEP',
                                            'gait_rough_terrain_middle_steps': 'GAIT RT MIDDLE STEPS',
                                            'gait_ramp_door_slope_up': 'GAIT RD SLOPE UP'})

    return sm_healthy
