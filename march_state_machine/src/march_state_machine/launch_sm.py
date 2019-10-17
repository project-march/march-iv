#!/usr/bin/env python
import smach
from smach import Sequence

from march_state_machine.states.EmptyState import EmptyState
from march_state_machine.states.LaunchSettingsState import LaunchSettingsState
from march_state_machine.states.LaunchState import LaunchState
from march_state_machine.states.WaitForParameterState import WaitForParameterState
from march_state_machine.states.WaitForRosControlState import WaitForRosControlState


##
# @brief The First sequence
# @details
# First the state machine launches packages that are needed to run before other packages
# For example safety is best to run first. The safety package checks that the system is behaving properly.
# @return The sequence.
def first_sequence():
    sequence = Sequence(
        outcomes=['succeeded', 'failed', 'exoskeleton', 'simulation'],
        connector_outcome='succeeded')
    with sequence:
        Sequence.add('SOUND SCHEDULER', LaunchState("march_sound_scheduler",
                                                    "march_sound_scheduler.launch",))
        Sequence.add('WAIT ON SETTINGS', LaunchSettingsState())
    return sequence


##
# @brief The Exoskeleton sequence.
# @details
# This sequence is specific for the Exoskeleton run configuration
# @return The sequence.
def exoskeleton_sequence():
    sequence = Sequence(
        outcomes=['succeeded', 'failed'],
        connector_outcome='succeeded')
    # The EmptyStates are placeholders for upcoming functional code
    # that we use to discuss and demo the launch feature.
    with sequence:
        Sequence.add('HARDWARE INTERFACE',
                     LaunchState("march_hardware_interface", "march4.launch",
                                 launch_is_optional_key="/exoskeleton/hardware_interface"
                                 ))
    return sequence


##
# @brief The Simulation sequence.
# @details
# This sequence is specific for the Simulation run configuration
# @return The sequence.
def simulation_sequence():
    sequence = Sequence(
        outcomes=['succeeded', 'failed'],
        connector_outcome='succeeded')
    with sequence:
        Sequence.add('GAZEBO',
                     LaunchState("march_simulation",
                                 "march_world.launch",
                                 launch_is_optional_key='',
                                 launch_arguments={
                                     "gazebo_ui": "/simulation/gazebo_ui"}
                                 ))
        Sequence.add('FAKE SENSOR DATA',
                     LaunchState("march_fake_sensor_data",
                                 "march_fake_sensor_data.launch",
                                 launch_is_optional_key="/simulation/fake_temperature_sensors"
                                 ))
        Sequence.add('WAIT FOR ROS_CONTROL', WaitForRosControlState())
    return sequence


##
# @brief The Default sequence.
# @details
# This sequence contains packages that are not run configuration specific
# @return The sequence.
def default_sequence():
    sequence = Sequence(
        outcomes=['succeeded', 'failed'],
        connector_outcome='succeeded')

    with sequence:
        Sequence.add('WAIT FOR JOINT_NAMES', WaitForParameterState(parameter="/march/joint_names"))
        Sequence.add('SAFETY', LaunchState("march_safety",
                                           "march_safety.launch",
                                           ))
        Sequence.add('RQT INPUT DEVICE',
                     LaunchState("march_rqt_input_device",
                                 "march_rqt_input_device.launch",
                                 launch_is_optional_key="/ui/rqt_input_device"))
        Sequence.add('RVIZ VISUALIZATION',
                     LaunchState("march_launch",
                                 "rviz.launch",
                                 launch_is_optional_key="/ui/rviz"))
        Sequence.add('RQT STATUS MONITOR',
                     LaunchState("march_launch",
                                 "rqt.launch",
                                 launch_is_optional_key="/ui/rqt_status_monitor"))
        Sequence.add('GAIT SELECTION', LaunchState("march_gait_selection",
                                                   "march_gait_selection.launch"))
        Sequence.add('GAIT SCHEDULER', LaunchState("march_gait_scheduler",
                                                   "gait_scheduler.launch"
                                                   ))
        Sequence.add('SERIAL CONNECTION',
                     LaunchState("march_launch",
                                 "serial_connection.launch",
                                 launch_is_optional_key="/input_device/real",
                                 launch_arguments={
                                     "tcp": "/input_device/wireless"}
                                 ))
    return sequence


##
# @brief Create the launch state machine.
# @details
# @return The launch state machine object.
def create():
    sm_launch = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm_launch:
        # Add states to the container

        smach.StateMachine.add('FIRST SEQUENCE', first_sequence(),
                               transitions={'exoskeleton': 'HARDWARE SEQUENCE',
                                            'simulation': 'SIMULATION SEQUENCE'})
        smach.StateMachine.add('HARDWARE SEQUENCE', exoskeleton_sequence(),
                               transitions={'succeeded': 'DEFAULT SEQUENCE'})
        smach.StateMachine.add('SIMULATION SEQUENCE', simulation_sequence(),
                               transitions={'succeeded': 'DEFAULT SEQUENCE'})
        smach.StateMachine.add('DEFAULT SEQUENCE', default_sequence())

    return sm_launch
