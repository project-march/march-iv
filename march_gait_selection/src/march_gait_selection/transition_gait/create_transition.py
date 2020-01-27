import rospy

from march_shared_classes.exceptions.gait_exceptions import GaitError, SubgaitNameNotFound
from march_shared_classes.gait.subgait import Subgait


SUBGAIT_NAME = 'transition_subgait'
GAIT_NAME = 'transition'
VERSION = 'default'


class TransitionSubgait(object):
    def __init__(self, gait_selection):
        self._gait_selection = gait_selection
        self._robot = gait_selection.robot

    def create_transition_subgait(self, old_gait_name, new_gait_name, new_subgait_name, old_subgait_name=None):
        """Create a new transition subgait object between two given sub-gaits.

        :param old_gait_name:
            The name of the old (or current) gait
        :param new_gait_name:
            The name of the new gait which must be executed after the old gait
        :param new_subgait_name:
            Name of the subgait in which the transition will occur
        :param old_subgait_name:
            Possible to give an custom old subgait name, if not the same gait name will be used as new_subgait_name

        :return:
            A populated Subgait object which holds the data to transition between given gaits using the subgait
        """
        old_gait = self._gait_selection[old_gait_name]
        new_gait = self._gait_selection[new_gait_name]

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

        old_subgait = old_gait[old_subgait_name]
        new_subgait = new_gait[new_subgait_name]

        if old_subgait is None:
            raise SubgaitNameNotFound(subgait_name=old_subgait_name)

        if new_subgait is None:
            raise SubgaitNameNotFound(subgait_name=new_subgait_name)

        old_subgait, new_subgait = self._scale_subgait_duration(old_subgait, new_subgait)

        joints, transition_positions, transition_velocities, transition_durations = \
            self._transition_trajectory(old_subgait, new_subgait)

        subgait_dict = self._to_subgait_dictionary(joints, transition_positions,
                                                   transition_velocities, transition_durations)

        return self._new_subgait(self._robot, subgait_dict)

    @staticmethod
    def _scale_subgait_duration(old_subgait, new_subgait):
        """Scale the subgaits to have matching setpoints and corresponding timestamps.

        :param old_subgait:
            The subgait which the transition is starting from
        :param new_subgait:
            The subgait which the transition will result in.

        :return:
            Same subgaits but with matching amount of setpoints and matching timestamps
        """
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
    def _transition_trajectory(old_subgait, new_subgait):
        """Calculate a transition trajectory which starts at the old gait and ends with the endpoints of the new gait.

        :param old_subgait:
            The subgait which the transition is starting from
        :param new_subgait:
            The subgait which the transition will result in.

        :returns:
             list of joints, velocities and positions corresponding to every joint at given timestamps and duration
        """
        joints = []
        transition_positions = []
        transition_velocities = []
        transition_durations = []

        total_transition_factor = len(new_subgait[0].setpoints)
        for transition_index in range(total_transition_factor):
            rospy.logwarn('transition index {ti} with timestamp {ts}'
                          .format(ti=str(transition_index), ts=str(new_subgait[0][transition_index].time)))

            positions = []
            velocities = []
            for old_joint in old_subgait.joints:
                new_joint = new_subgait.get_joint(old_joint.name)

                if len(old_joint) != len(new_joint):
                    rospy.logerr('Amount of setpoints is inconsistent between subgait {og} of gait {fg} and {sg}'
                                 .format(og=old_subgait.subgait_name, fg=old_subgait.gait_name,
                                         sg=new_subgait.gait_name))

                joints.append(new_joint.name)

                new_setpoint_factor = 1.0 / (total_transition_factor - 1) * transition_index
                old_setpoint_factor = 1.0 - new_setpoint_factor

                old_setpoint = old_joint[transition_index]
                new_setpoint = new_joint[transition_index]

                positions.append((old_setpoint.position * old_setpoint_factor)
                                 + (new_setpoint.position * new_setpoint_factor))
                velocities.append((old_setpoint.velocity * old_setpoint_factor)
                                  + (new_setpoint.velocity * new_setpoint_factor))

                trans_pos = (old_setpoint.position * old_setpoint_factor) + \
                            (new_setpoint.position * new_setpoint_factor)

                rospy.logwarn('old factor: {of} \t new factor {nf} \t old: {op} \t transition: {tp} \t new: {np} '
                              .format(of=str(old_setpoint_factor), nf=str(new_setpoint_factor),
                                      op=str(old_setpoint.position), tp=str(trans_pos), np=str(new_setpoint.position)))

            transition_positions.append(positions)
            transition_velocities.append(velocities)
            transition_durations.append(new_setpoint.time)

        return joints, transition_positions, transition_velocities, transition_durations

    @staticmethod
    def _to_subgait_dictionary(joints, positions_list, velocity_list, durations):
        """Format data to create a Subgait class object later on.

        :param joints:
            List of the used joints in the same order as in the setpoints
        :param positions_list:
            List of joint positions corresponding to the timestamps
        :param velocity_list:
            List of joint velocities corresponding to the timestamps
        :param durations:
            List of durations of the setpoints

        :return:
            Dictionary in specific format used to create a Subgait object (format matching .subgait files)
        """
        ros_duration = None
        trajectory = {}

        points = []
        for positions, velocities, duration in zip(positions_list, velocity_list, durations):
            ros_duration = rospy.Duration.from_sec(duration)
            point = {'positions': positions,
                     'velocities': velocities,
                     'time_from_start': {'secs': ros_duration.secs, 'nsecs': ros_duration.nsecs},
                     }
            points.append(point)

            rospy.logwarn('Transition timestamp: ' + str({'secs': ros_duration.secs, 'nsecs': ros_duration.nsecs}))
            rospy.logwarn('Transition position: ' + str(positions))

        trajectory['joint_names'] = joints
        trajectory['points'] = points

        return {'trajectory': trajectory, 'duration': {'secs': ros_duration.secs, 'nsecs': ros_duration.nsecs}}

    @staticmethod
    def _new_subgait(robot, subgait_dictionary):
        """Create a subgait object using a subgait dictionary of a .subgait or generated dictionary.

        :param robot:
            The robot to execute the gaits
        :param subgait_dictionary:
            The data of the subgait packed in the .subgait file format

        :return:
            A populated Subgait object
        """
        return Subgait.from_dict(robot, subgait_dictionary, GAIT_NAME, SUBGAIT_NAME, VERSION)
