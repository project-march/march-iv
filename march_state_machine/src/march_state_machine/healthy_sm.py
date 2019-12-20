import rospy
import smach
from std_srvs.srv import Empty, EmptyRequest

from .gaits import ramp_down_sm
from .state_machines.slope_state_machine import SlopeStateMachine
from .state_machines.step_state_machine import StepStateMachine
from .state_machines.tilted_path_straight_start_left_state_machine import TiltedPathStraightStartLeftStateMachine
from .state_machines.walk_state_machine import WalkStateMachine
from .states.idle_state import IdleState


class HealthyStart(smach.State):
    def __init__(self):
        super(HealthyStart, self).__init__(outcomes=['succeeded'])

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
                               transitions={'home_sit': 'HOME SIT', 'home_stand': 'HOME STAND'})

        # Movement states
        smach.StateMachine.add('HOME SIT', StepStateMachine('home', ['home_sit']),
                               transitions={'succeeded': 'SITTING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('HOME STAND', StepStateMachine('home', ['home_stand']),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT WALK', WalkStateMachine('walk'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT WALK SMALL', WalkStateMachine('walk_small'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIT', StepStateMachine('sit', ['sit_down', 'sit_home']),
                               transitions={'succeeded': 'SITTING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT STAND', StepStateMachine('stand', ['prepare_stand_up', 'stand_up']),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SINGLE STEP SMALL', StepStateMachine('single_step_small'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SINGLE STEP NORMAL', StepStateMachine('single_step_normal'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP LEFT', StepStateMachine('side_step_left',
                                                                       ['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP RIGHT', StepStateMachine('side_step_right'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP LEFT SMALL', StepStateMachine('side_step_left_small',
                                                                             ['left_open', 'right_close']),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SIDE STEP RIGHT SMALL', StepStateMachine('side_step_right_small'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SOFA SIT', StepStateMachine('sofa_sit', ['sit_down', 'sit_home']),
                               transitions={'succeeded': 'SOFA SITTING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT SOFA STAND', StepStateMachine('sofa_stand', ['prepare_stand_up', 'stand_up']),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT STAIRS UP', WalkStateMachine('stairs_up'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT STAIRS DOWN', WalkStateMachine('stairs_down'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        # RT stands for Rough Terrain
        smach.StateMachine.add('GAIT RT HIGH STEP', StepStateMachine('rough_terrain_high_step'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT RT MIDDLE STEPS', StepStateMachine('rough_terrain_middle_steps',
                                                                        ['right_open', 'left_swing',
                                                                         'right_swing', 'left_close']),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        # RD stands for Ramp and Door
        smach.StateMachine.add('GAIT RD SLOPE UP', SlopeStateMachine('ramp_door_slope_up'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT RD RAMP DOWN', ramp_down_sm.create(),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        # TP stands for Tilted Path
        smach.StateMachine.add('GAIT TP STRAIGHT START RIGHT', StepStateMachine('tilted_path_straight_start_right'),
                               transitions={'succeeded': 'STANDING', 'failed': 'UNKNOWN'})

        smach.StateMachine.add('GAIT TP STRAIGHT START LEFT',
                               TiltedPathStraightStartLeftStateMachine('tilted_path_straight_start_left'),
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
                                                               'gait_walk_small', 'gait_rough_terrain_high_step',
                                                               'gait_rough_terrain_middle_steps',
                                                               'gait_ramp_door_slope_up', 'gait_ramp_door_slope_down',
                                                               'gait_tilted_path_straight_start_right',
                                                               'gait_tilted_path_straight_start_left',
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
                                            'gait_walk_small': 'GAIT WALK SMALL',
                                            'gait_rough_terrain_high_step': 'GAIT RT HIGH STEP',
                                            'gait_rough_terrain_middle_steps': 'GAIT RT MIDDLE STEPS',
                                            'gait_ramp_door_slope_up': 'GAIT RD SLOPE UP',
                                            'gait_ramp_door_slope_down': 'GAIT RD RAMP DOWN',
                                            'gait_tilted_path_straight_start_right': 'GAIT TP STRAIGHT START RIGHT',
                                            'gait_tilted_path_straight_start_left': 'GAIT TP STRAIGHT START LEFT'})

    return sm_healthy
