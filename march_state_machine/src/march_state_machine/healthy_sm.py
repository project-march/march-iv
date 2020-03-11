import rospy
import smach
from std_srvs.srv import Empty, EmptyRequest

from march_shared_resources.srv import CurrentState, PossibleGaits

from .gaits import slope_down_sm
from .gaits import tilted_path_flexed_knee_step_sm
from .gaits import tilted_path_sideways_end_sm
from .gaits import tilted_path_sideways_start_sm
from .gaits import tilted_path_straight_sm
from .state_machines.slope_state_machine import SlopeStateMachine
from .state_machines.step_state_machine import StepStateMachine
from .state_machines.transition_state_machine import StateMachineWithTransition
from .state_machines.walk_state_machine import WalkStateMachine
from .states.idle_state import IdleState


class HealthyStart(smach.State):
    def __init__(self):
        super(HealthyStart, self).__init__(outcomes=['succeeded'], input_keys=['sounds'], output_keys=['sounds'])

    def execute(self, userdata):
        if rospy.get_param('~unpause', False):
            unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause.wait_for_service()
            unpause(EmptyRequest())
        rospy.loginfo('March is fully operational')
        if userdata.sounds:
            # Sleep is necessary to wait for the soundplay node
            rospy.sleep(1.0)
            userdata.sounds.play('start')
        return 'succeeded'


class HealthyStateMachine(smach.StateMachine):

    def __init__(self):
        super(HealthyStateMachine, self).__init__(outcomes=['succeeded', 'failed', 'preempted'], input_keys=['sounds'],
                                                  output_keys=['sounds'])

        rospy.Service('state_machine/get_possible_gaits', PossibleGaits, self.get_possible_gaits)

        rospy.Service('state_machine/current_states', CurrentState, self.get_current_states)

        self.open()
        self.add_auto('START', HealthyStart(), connector_outcomes=['succeeded'])
        self.add('UNKNOWN', IdleState(outcomes=['home_sit', 'home_stand', 'failed', 'preempted']),
                 transitions={'home_sit': 'HOME SIT', 'home_stand': 'HOME STAND'})

        self.add_state('HOME SIT', StepStateMachine('home', ['home_sit']), 'SITTING')
        self.add_state('HOME STAND', StepStateMachine('home', ['home_stand']), 'STANDING')

        walking_state_machine = StateMachineWithTransition(['walk_small', 'walk', 'walk_large'])
        walking_state_machine.add('walk_small', WalkStateMachine('walk_small', is_transition_active=True))
        walking_state_machine.add('walk', WalkStateMachine('walk', is_transition_active=True), default_start=True)
        walking_state_machine.add('walk_large', WalkStateMachine('walk_large', is_transition_active=True))

        self.add_state('GAIT WALK', walking_state_machine, 'STANDING')

        self.add_state('GAIT WALK SMALL', WalkStateMachine('walk_small'), 'STANDING')
        self.add_state('GAIT WALK LARGE', WalkStateMachine('walk_large'), 'STANDING')

        self.add_state('GAIT SIT', StepStateMachine('sit', ['sit_down', 'sit_home']), 'SITTING')
        self.add_state('GAIT STAND', StepStateMachine('stand', ['prepare_stand_up', 'stand_up']), 'STANDING')

        self.add_state('GAIT SINGLE STEP SMALL', StepStateMachine('single_step_small'), 'STANDING')
        self.add_state('GAIT SINGLE STEP NORMAL', StepStateMachine('single_step_normal'), 'STANDING')

        self.add_state('GAIT SIDE STEP LEFT',
                       StepStateMachine('side_step_left', ['left_open', 'right_close']),
                       'STANDING')
        self.add_state('GAIT SIDE STEP LEFT SMALL',
                       StepStateMachine('side_step_left_small', ['left_open', 'right_close']),
                       'STANDING')

        self.add_state('GAIT SIDE STEP RIGHT', StepStateMachine('side_step_right'), 'STANDING')
        self.add_state('GAIT SIDE STEP RIGHT SMALL', StepStateMachine('side_step_right_small'), 'STANDING')

        self.add_state('GAIT SOFA SIT', StepStateMachine('sofa_sit', ['sit_down', 'sit_home']), 'SOFA SITTING')
        self.add('SOFA SITTING', IdleState(outcomes=['gait_sofa_stand', 'preempted']),
                 transitions={'gait_sofa_stand': 'GAIT SOFA STAND'})
        self.add_state('GAIT SOFA STAND', StepStateMachine('sofa_stand', ['prepare_stand_up', 'stand_up']), 'STANDING')

        self.add_state('GAIT STAIRS UP', WalkStateMachine('stairs_up'), 'STANDING')
        self.add_state('GAIT STAIRS DOWN', WalkStateMachine('stairs_down'), 'STANDING')
        self.add_state('GAIT STAIRS UP SINGLE STEP', StepStateMachine('stairs_up_single_step'), 'STANDING')
        self.add_state('GAIT STAIRS DOWN SINGLE STEP', StepStateMachine('stairs_down_single_step'), 'STANDING')

        # RT stands for Rough Terrain
        self.add_state('GAIT RT HIGH STEP', StepStateMachine('rough_terrain_high_step'), 'STANDING')
        self.add_state('GAIT RT MIDDLE STEPS', StepStateMachine('rough_terrain_middle_steps',
                                                                ['right_open', 'left_swing',
                                                                 'right_swing', 'left_close']),
                       'STANDING')

        # These are three separate steps for the middle steps of the RT, as an alternative to the middle steps above.
        self.add_state('GAIT RT FIRST MIDDLE STEP', StepStateMachine('rough_terrain_first_middle_step'), 'STANDING')
        self.add_state('GAIT RT SECOND MIDDLE STEP', StepStateMachine('rough_terrain_second_middle_step'), 'STANDING')
        self.add_state('GAIT RT THIRD MIDDLE STEP', StepStateMachine('rough_terrain_third_middle_step'), 'STANDING')

        # RD stands for Ramp and Door
        self.add_state('GAIT RD SLOPE UP', SlopeStateMachine('ramp_door_slope_up'), 'STANDING')
        self.add_state('GAIT RD SLOPE DOWN', slope_down_sm.create(), 'STANDING')

        # TP stands for Tilted Path
        self.add_state('GAIT TP STRAIGHT', tilted_path_straight_sm.create(), 'STANDING')

        self.add_state('GAIT TP SIDEWAYS START', tilted_path_sideways_start_sm.create(), 'STANDING')
        self.add_state('GAIT TP SIDEWAYS END', tilted_path_sideways_end_sm.create(), 'STANDING')

        self.add_state('GAIT TP FLEXED KNEE STEP', tilted_path_flexed_knee_step_sm.create(), 'STANDING')

        self.add('SITTING', IdleState(outcomes=['gait_stand', 'preempted']),
                 transitions={'gait_stand': 'GAIT STAND'})
        self.add('STANDING', IdleState(outcomes=['gait_sit', 'gait_walk', 'gait_walk_small', 'gait_single_step_small',
                                                 'gait_walk_large', 'gait_single_step_normal', 'gait_side_step_left',
                                                 'gait_side_step_right', 'gait_side_step_left_small',
                                                 'gait_side_step_right_small', 'gait_sofa_sit',
                                                 'gait_stairs_up', 'gait_stairs_down',
                                                 'gait_stairs_up_single_step', 'gait_stairs_down_single_step',
                                                 'gait_walk_small', 'gait_rough_terrain_high_step',
                                                 'gait_rough_terrain_middle_steps',
                                                 'gait_rough_terrain_first_middle_step',
                                                 'gait_rough_terrain_second_middle_step',
                                                 'gait_rough_terrain_third_middle_step',
                                                 'gait_ramp_door_slope_up', 'gait_ramp_door_slope_down',
                                                 'gait_tilted_path_straight_start',
                                                 'gait_tilted_path_first_start',
                                                 'gait_tilted_path_first_end',
                                                 'gait_tilted_path_flexed_knee_step',
                                                 'preempted']),
                 transitions={'gait_sit': 'GAIT SIT',
                              'gait_walk': 'GAIT WALK',
                              'gait_walk_small': 'GAIT WALK SMALL',
                              'gait_walk_large': 'GAIT WALK LARGE',
                              'gait_single_step_small': 'GAIT SINGLE STEP SMALL',
                              'gait_single_step_normal': 'GAIT SINGLE STEP NORMAL',
                              'gait_side_step_left': 'GAIT SIDE STEP LEFT',
                              'gait_side_step_right': 'GAIT SIDE STEP RIGHT',
                              'gait_side_step_left_small': 'GAIT SIDE STEP LEFT SMALL',
                              'gait_side_step_right_small': 'GAIT SIDE STEP RIGHT SMALL',
                              'gait_sofa_sit': 'GAIT SOFA SIT',
                              'gait_stairs_up': 'GAIT STAIRS UP',
                              'gait_stairs_down': 'GAIT STAIRS DOWN',
                              'gait_stairs_up_single_step': 'GAIT STAIRS UP SINGLE STEP',
                              'gait_stairs_down_single_step': 'GAIT STAIRS DOWN SINGLE STEP',
                              'gait_rough_terrain_high_step': 'GAIT RT HIGH STEP',
                              'gait_rough_terrain_middle_steps': 'GAIT RT MIDDLE STEPS',
                              'gait_rough_terrain_first_middle_step': 'GAIT RT FIRST MIDDLE STEP',
                              'gait_rough_terrain_second_middle_step': 'GAIT RT SECOND MIDDLE STEP',
                              'gait_rough_terrain_third_middle_step': 'GAIT RT THIRD MIDDLE STEP',
                              'gait_ramp_door_slope_up': 'GAIT RD SLOPE UP',
                              'gait_ramp_door_slope_down': 'GAIT RD SLOPE DOWN',
                              'gait_tilted_path_straight_start': 'GAIT TP STRAIGHT',
                              'gait_tilted_path_first_start': 'GAIT TP SIDEWAYS START',
                              'gait_tilted_path_first_end': 'GAIT TP SIDEWAYS END',
                              'gait_tilted_path_flexed_knee_step': 'GAIT TP FLEXED KNEE STEP'})
        self.close()

    def add_state(self, label, state, succeeded):
        """Adds a state to the healthy state machine.

        The healthy state machine should be opened before using this method.

        :type label: str
        :param label: name of the state
        :type state: smach.State
        :param state: State (or statemachine to be added)
        :type: succeeded: str
        :param succeeded: name of the state that the given state should transition to once succeeded
        """
        self.assert_opened()
        self.add(label, state, transitions={'succeeded': succeeded, 'failed': 'UNKNOWN'})

    def get_possible_gaits(self, _req):
        """Returns the possible gaits from the current state.

        :type _req: march_shared_resources.srv.PossibleGaitsRequest
        :rtype march_shared_resources.srv.PossibleGaitsResponse
        """
        non_gaits = ['succeeded', 'failed', 'preempted', 'aborted']
        gaits = []
        if self.is_running():
            gaits = [k for k in self._current_transitions.keys() if k not in non_gaits]
            # If no possible gaits are on this level, try one state machine deeper
            if not gaits:
                if self._current_state._current_transitions:
                    gaits = [k for k in self._current_state._current_transitions.keys() if k not in non_gaits]

        return {'gaits': gaits}

    def get_current_states(self, _req):
        state = self.get_active_states()[0]
        state_type = str(type(self[state]))
        return state_type, state
