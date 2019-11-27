import rospy
import yaml

from limits import Limits
from joint_trajectory import JointTrajectory

from march_shared_resources import msg as march_msg
from trajectory_msgs import msg as trajectory_msg


class Subgait(object):
    joint_class = JointTrajectory

    def __init__(self, joints, duration, gait_type='walk_like',
                 gait_name='Walk', subgait_name='right_open', version='First try', description='Just a simple gait'):
        # Set gait_type to walk_like if an old file with no gait_type is opened
        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name
        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    @classmethod
    def from_file(cls, robot, file_name, *args):
        if file_name is None or file_name == "":
            return None
        try:
            gait_name = file_name.split("/")[-3]
            subgait_name = file_name.split("/")[-2]
            version = file_name.split("/")[-1].replace(".subgait", "")
            subgait_dict = yaml.load(open(file_name), Loader=yaml.SafeLoader)
            if 'gait_type' not in subgait_dict:
                subgait_dict['gait_type'] = 'walk_like'
        except Exception as e:
            rospy.logerr("Error occured in subgait: {}, {} ".format(type(e), e))
            return None
        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version, *args)

    @classmethod
    def from_dict(cls, robot, subgait_dict, gait_name, subgait_name, version, *args):
        if robot is None:
            rospy.logerr("Cannot create gait without a loaded robot.")
            return None

        joint_trajectory = subgait_dict['trajectory']
        duration = rospy.Duration(subgait_dict['duration']['secs'], subgait_dict['duration']['nsecs']).to_sec()
        joint_list = []
        for joint_name in joint_trajectory['joint_names']:
            urdf_joint = cls._get_joint_from_urdf(robot, joint_name)
            if urdf_joint is None:
                rospy.logwarn("Not all joints in gait are in robot.")
                continue

            limits = Limits(urdf_joint.safety_controller.soft_lower_limit,
                            urdf_joint.safety_controller.soft_upper_limit,
                            urdf_joint.limit.velocity)

            joint_list.append(cls.joint_class.from_dict(subgait_dict, joint_name, limits, duration, *args))

        return cls(joint_list, duration, subgait_dict['gait_type'], gait_name, subgait_name,
                   version, subgait_dict['description'])

    @staticmethod
    def _get_joint_from_urdf(robot, joint_name):
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    def to_subgait_msg(self):
        # Name and version will be empty as it's stored in the filename.
        subgait_msg = march_msg.Subgait()

        subgait_msg.gait_type = self.gait_type
        subgait_msg.trajectory = self._to_joint_trajectory_msg()
        subgait_msg.setpoints = self.to_setpoints()
        subgait_msg.description = str(self.description)

        subgait_msg.duration = rospy.Duration.from_sec(self.duration)
        return subgait_msg

    def _to_joint_trajectory_msg(self):
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
                    rospy.logwarn("Time mismatch in joint {} at timestamp {}, "
                                  "got time {}".format(joint.name, timestamp, interpolated_setpoint.time))
                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)
            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    def to_setpoints(self):
        user_defined_setpoints = []
        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            user_defined_setpoint = march_msg.Setpoint()
            user_defined_setpoint.time_from_start = rospy.Duration.from_sec(timestamp)
            for joint in self.joints:
                for setpoint in joint.setpoints:
                    if setpoint.time == timestamp:
                        user_defined_setpoint.joint_names.append(joint.name)
            user_defined_setpoints.append(user_defined_setpoint)
        return user_defined_setpoints

    def get_unique_timestamps(self):
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        rospy.logerr('Joint with name ' + name + ' does not exist in gait ' + self.gait_name)
        return None

    def __getitem__(self, index):
        return self.joints[index]

    def __len__(self):
        return len(self.joints)
