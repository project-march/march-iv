from copy import deepcopy

from march_shared_classes.exceptions.gait_exceptions import GaitError, SubgaitNameNotFound, TransitionError
from march_shared_classes.gait.gait import Gait
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait


class TransitionSubgait(Subgait):
    def __init__(self, joints, duration,
                 gait_type='walk_like', gait_name='Transition',
                 subgait_name='Transition_subgait', version='Default',
                 description='The subgait used to transition between two subgaits'):

        Subgait.__init__(self, joints, duration, gait_type, gait_name, subgait_name, version, description)

    @classmethod
    def from_subgait_names(cls, gait_selection, old_gait_name, new_gait_name, new_subgait_name, old_subgait_name=None):
        """Create a new transition subgait object between two given sub-gaits.

        :param gait_selection:
            The gait selection object which holds all the information about the gaits and subgaits
        :param old_gait_name:
            The name of the old (or current) gait
        :param new_gait_name:
            The name of the new gait which must be executed after the old gait
        :param new_subgait_name:
            Name of the subgait in which the transition will occur
        :param old_subgait_name:
            Possible to give an custom old subgait name, if not the same gait name will be used as new_subgait_name

        :return:
            A populated TransitionSubgait object which holds the data to transition between given gaits
        """
        old_subgait, new_subgait = cls._get_subgaits(gait_selection, old_gait_name, new_gait_name,
                                                     new_subgait_name, old_subgait_name)

        old_subgait, new_subgait = cls._scale_timestamps_subgaits(old_subgait, new_subgait)

        old_subgait, new_subgait = cls._equalize_amount_of_setpoints(old_subgait, new_subgait)

        transition_joints = cls._transition_joints(gait_selection.robot, old_subgait, new_subgait)
        transition_duration = new_subgait.duration

        transition_subgait = cls(transition_joints, transition_duration)
        cls._validate_transition_gait(old_subgait, transition_subgait, new_subgait)

        return transition_subgait

    @staticmethod
    def _get_subgaits(gait_selection, old_gait_name, new_gait_name, new_subgait_name, old_subgait_name):
        """Use the subgait names and the parsed subgaits in the gait selection object to get the subgait objects."""
        old_gait = gait_selection[old_gait_name]
        new_gait = gait_selection[new_gait_name]

        if old_gait is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=old_gait_name))

        if new_gait is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=new_gait_name))

        if old_gait.to_subgaits_names != new_gait.to_subgaits_names:
            raise GaitError(msg='To_subgait list do not match between gait: {cg} and gait: {ng}'
                            .format(cg=old_gait_name, ng=new_gait_name))

        if old_gait.from_subgaits_names != new_gait.from_subgaits_names:
            raise GaitError(msg='From_subgait list do not match between gait: {cg} and gait: {ng}'
                            .format(cg=old_gait_name, ng=new_gait_name))

        old_subgait_name = new_subgait_name if old_subgait_name is None else old_subgait_name

        old_subgait = deepcopy(old_gait[old_subgait_name])
        new_subgait = deepcopy(new_gait[new_subgait_name])

        if old_subgait is None:
            raise SubgaitNameNotFound(subgait_name=old_subgait_name)

        if new_subgait is None:
            raise SubgaitNameNotFound(subgait_name=new_subgait_name)

        return old_subgait, new_subgait

    @staticmethod
    def _equalize_amount_of_setpoints(old_subgait, new_subgait):
        """Equalize the subgaits to have matching amount of setpoints on all the timestamps."""
        timestamps = sorted(set(old_subgait.get_unique_timestamps() + new_subgait.get_unique_timestamps()))

        for old_joint in old_subgait.joints:
            new_joint = new_subgait.get_joint(old_joint.name)

            old_joint_setpoints = []
            new_joint_setpoints = []
            for timestamp in timestamps:
                old_joint_setpoints.append(old_joint.get_interpolated_setpoint(timestamp))
                new_joint_setpoints.append(new_joint.get_interpolated_setpoint(timestamp))

            old_joint.setpoints = old_joint_setpoints
            new_joint.setpoints = new_joint_setpoints

        return old_subgait, new_subgait

    @staticmethod
    def _scale_timestamps_subgaits(old_subgait, new_subgait):
        """Scale all the setpoint to match the duration in both subgaits."""
        old_duration = old_subgait.duration
        new_duration = new_subgait.duration

        for joint in old_subgait.joints:
            for setpoint in joint.setpoints:
                setpoint.time = setpoint.time / old_duration * new_duration

        return old_subgait, new_subgait

    @staticmethod
    def _transition_joints(robot, old_subgait, new_subgait):
        """Calculate a transition trajectory which starts at the old gait and ends with the endpoints of the new gait.

        :returns:
             list of joints which hold the transition setpoints including position, velocity and duration
        """
        joints = []
        for old_joint in old_subgait.joints:

            joint_name = old_joint.name
            new_joint = new_subgait.get_joint(joint_name)
            limits = TransitionSubgait._get_limits(robot, joint_name)

            setpoints = []
            number_setpoints = len(new_subgait[0].setpoints)
            for transition_index in range(number_setpoints):
                new_setpoint_factor = transition_index / (number_setpoints - 1.0)
                old_setpoint_factor = 1.0 - new_setpoint_factor

                old_setpoint = old_joint[transition_index]
                new_setpoint = new_joint[transition_index]

                position = (old_setpoint.position * old_setpoint_factor) + (new_setpoint.position * new_setpoint_factor)
                velocity = (old_setpoint.velocity * old_setpoint_factor) + (new_setpoint.velocity * new_setpoint_factor)
                time = new_setpoint.time

                setpoints.append(Setpoint(time, position, velocity))

            joints.append(JointTrajectory(joint_name, limits, setpoints, old_joint.duration))

        return joints

    @staticmethod
    def _get_limits(robot, joint_name):
        """Use the parsed robot to get the defined joint limits."""
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return Limits(urdf_joint.safety_controller.soft_lower_limit,
                              urdf_joint.safety_controller.soft_upper_limit,
                              urdf_joint.limit.velocity)

        raise TransitionError('No robot selected for create the transition gait')

    @staticmethod
    def _validate_transition_gait(old_subgait, transition_subgait, new_subgait):

        subgaits = [old_subgait, transition_subgait, new_subgait]
        from_subgaits = ['start', old_subgait.subgait_name, transition_subgait.subgait_name, new_subgait.subgait_name]
        to_subgaits = [old_subgait.subgait_name, transition_subgait.subgait_name, new_subgait.subgait_name, 'end']

        try:
            Gait('transition', subgaits, from_subgaits, to_subgaits)
        except Exception as error:
            TransitionError('Error when creating transition: {er}'.format(er=error))
