
from joint_trajectory import JointTrajectory
from limits import Limits
import rospy
from trajectory_msgs import msg as trajectory_msg
import yaml

from march_shared_classes.exceptions.gait_exceptions import NonValidGaitContent
from march_shared_resources import msg as march_msg


class Subgait(object):
    """Base class for usage of the defined sub gaits."""

    joint_class = JointTrajectory

    def __init__(self, joints, duration, gait_type='walk_like', gait_name='Walk', subgait_name='right_open',
                 version='First try', description='Just a simple gait'):

        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name

        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    @classmethod
    def from_file(cls, robot, file_name, *args):
        """Extract sub gait data of the given yaml.

        :param robot:
            The robot corresponding to the given sub-gait file
        :param file_name:
            The .yaml file name of the sub gait

        :returns
            A populated Subgait object
        """
        if file_name is None or file_name == '':
            return None

        try:
            gait_name = file_name.split('/')[-3]
            subgait_name = file_name.split('/')[-2]
            version = file_name.split('/')[-1].replace('.subgait', '')

            with open(file_name, 'r') as yaml_file:
                subgait_dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)

        except Exception as e:
            rospy.logerr('Error occurred in subgait: {te}, {er} '.format(te=type(e), er=e))
            return None

        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version, *args)

    @classmethod
    def from_dict(cls, robot, subgait_dict, gait_name, subgait_name, version, *args):
        """List parameters from the yaml file in organized lists.

        :param robot:
            The robot corresponding to the given sub-gait file
        :param subgait_dict:
            The dictionary extracted from the yaml file
        :param gait_name:
            The name of the parent gait
        :param subgait_name:
            The name of the child (sub)gait
        :param version:
            The version of the yaml file

        :returns
            A populated Subgait object
        """
        if robot is None:
            rospy.logerr('Cannot create gait without a loaded robot.')
            return None

        joint_trajectory = subgait_dict['trajectory']
        duration = rospy.Duration(subgait_dict['duration']['secs'], subgait_dict['duration']['nsecs']).to_sec()

        joint_list = []
        for joint_name in joint_trajectory['joint_names']:
            urdf_joint = cls._get_joint_from_urdf(robot, joint_name)
            if urdf_joint is None:
                rospy.logwarn('Not all joints in gait are in robot.')
                continue

            limits = Limits(urdf_joint.safety_controller.soft_lower_limit,
                            urdf_joint.safety_controller.soft_upper_limit,
                            urdf_joint.limit.velocity)

            joint_list.append(cls.joint_class.from_dict(subgait_dict, joint_name, limits, duration, *args))

        subgait_type = subgait_dict['gait_type'] if subgait_dict.get('gait_type') else ''
        subgait_description = subgait_dict['description'] if subgait_dict.get('description') else ''

        return cls(joint_list, duration, subgait_type, gait_name, subgait_name, version, subgait_description)

    @staticmethod
    def _get_joint_from_urdf(robot, joint_name):
        """Get the name of the robot joint corresponding with the joint in the subgait."""
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    def _to_joint_trajectory_msg(self):
        """Create trajectory msg for the publisher.

        :returns
            a ROS msg for the joint trajectory
        """
        joint_trajectory_msg = trajectory_msg.JointTrajectory()

        timestamps = self.get_unique_timestamps()

        for joint in self.joints:
            joint_trajectory_msg.joint_names.append(joint.name)

        for timestamp in timestamps:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = rospy.Duration(timestamp)

            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)
                if interpolated_setpoint.time != timestamp:
                    rospy.logwarn('Time mismatch in joint {jn} at timestamp {ts}, '
                                  'got time {ti}'.format(jn=joint.name, ts=timestamp, ti=interpolated_setpoint.time))

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    def _to_setpoints_msg(self):
        """Define setpoints that correspond with the given timestamps."""
        timestamps = self.get_unique_timestamps()

        user_defined_setpoints = []
        for timestamp in timestamps:
            user_defined_setpoint = march_msg.Setpoint()
            user_defined_setpoint.time_from_start = rospy.Duration.from_sec(timestamp)

            for joint in self.joints:
                for setpoint in joint.setpoints:
                    if setpoint.time == timestamp:
                        user_defined_setpoint.joint_names.append(joint.name)

            user_defined_setpoints.append(user_defined_setpoint)

        return user_defined_setpoints

    def to_subgait_msg(self):
        """Convert class attribute values back to ROS msg (necessary for publisher)."""
        subgait_msg = march_msg.Subgait()

        subgait_msg.name = self.subgait_name
        subgait_msg.gait_type = self.gait_type
        subgait_msg.trajectory = self._to_joint_trajectory_msg()
        subgait_msg.setpoints = self._to_setpoints_msg()
        subgait_msg.description = self.description
        subgait_msg.version = self.version
        subgait_msg.duration = rospy.Duration.from_sec(self.duration)

        return subgait_msg

    def validate_subgait_transition(self, next_subgait):
        """Validate the trajectory transition of this gait to a given gait.

        :param next_subgait:
            The subgait subsequently to this gait (not the previous one!)

        :returns:
            True if trajectory transition correct else False
        """
        from_subgait_joint_names = set(self.get_joint_names())
        to_subgait_joint_names = set(next_subgait.get_joint_names())

        if from_subgait_joint_names != to_subgait_joint_names:
            raise NonValidGaitContent(msg='Structure of joints does not match between subgait {fn} and subgait {tn}'
                                      .format(fn=self.subgait_name, tn=next_subgait.subgait_name))

        for joint_name in to_subgait_joint_names:
            from_joint = self.get_joint(joint_name)
            to_joint = next_subgait.get_joint(joint_name)
            if not from_joint.validate_joint_transition(to_joint):
                return False

        return True

    def get_unique_timestamps(self):
        """The timestamp that is unique to a setpoint."""
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name):
        """Get joint object with given name or index."""
        return next(joint for joint in self.joints if joint.name == name)

    def get_joint_names(self):
        """Get the names of all the joints existing in the joint list."""
        return [joint.name for joint in self.joints]

    def __getitem__(self, index):
        return self.joints[index]

    def __len__(self):
        return len(self.joints)
