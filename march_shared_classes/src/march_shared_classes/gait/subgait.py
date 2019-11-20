import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from march_shared_resources.msg import Setpoint
from limits import Limits
from joint_trajectory import JointTrajectory
import yaml
from rospy_message_converter import message_converter


class Subgait:
    joint_class = JointTrajectory

    def __init__(self, joints, duration, gait_type='walk_like',
                 gait_name='Walk', subgait_name='right_open', version='First try', description='Just a simple gait'):
        # Set gait_type to walk_like if an old file with no gait_type is opened
        if gait_type == '':
            gait_type = 'walk_like'

        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name
        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    @classmethod
    def from_file(cls, robot, file_name):
        if file_name is None or file_name == "":
            return None
        try:
            gait_name = file_name.split("/")[-3]
            subgait_name = file_name.split("/")[-2]
            version = file_name.split("/")[-1].replace(".subgait", "")
            march_subgait_yaml = yaml.load(open(file_name), Loader=yaml.SafeLoader)
            march_subgait = message_converter.convert_dictionary_to_ros_message(
                'march_shared_resources/Subgait', march_subgait_yaml)
        except Exception as e:
            rospy.logerr(str(e))
            return None
        return cls.from_msg(robot, march_subgait, gait_name, subgait_name, version)

    @classmethod
    def from_msg(cls, robot, subgait_msg, gait_name, subgait_name, version):
        if robot is None:
            rospy.logerr("Cannot create gait without a loaded robot.")
            return None

        joint_trajectory = subgait_msg.trajectory
        duration = rospy.Duration(subgait_msg.duration.secs, subgait_msg.duration.nsecs).to_sec()
        joint_list = []
        for joint_name in joint_trajectory.joint_names:
            urdf_joint = cls._get_joint_from_urdf(robot, joint_name)
            if urdf_joint is None:
                rospy.logwarn("Not all joints in gait are in robot.")
                continue

            limits = Limits(urdf_joint.safety_controller.soft_lower_limit,
                            urdf_joint.safety_controller.soft_upper_limit,
                            urdf_joint.limit.velocity)

            joint_list.append(cls.joint_class.from_msg(subgait_msg, joint_name, limits, duration))

        return cls(joint_list, duration, subgait_msg.gait_type, gait_name, subgait_name,
                   version, subgait_msg.description)

    @staticmethod
    def _get_joint_from_urdf(robot, joint_name):
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    def to_joint_trajectory(self):
        joint_trajectory = JointTrajectory()

        timestamps = self.get_unique_timestamps()

        for joint in self.joints:
            joint_trajectory.joint_names.append(joint.name)

        for timestamp in timestamps:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = rospy.Duration(timestamp)
            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)

                if interpolated_setpoint.time != timestamp:
                    rospy.logerr('Time mismatch in joint ' + joint.name + ' at timestamp ' + timestamp)
                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)
            joint_trajectory.points.append(joint_trajectory_point)

        return joint_trajectory

    def to_setpoints(self):
        user_defined_setpoints = []
        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            user_defined_setpoint = Setpoint()
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
        for i in range(0, len(self.joints)):
            if self.joints[i].name == name:
                return self.joints[i]
        rospy.logerr('Joint with name ' + name + ' does not exist in gait ' + self.gait_name)
        return None

    def set_gait_type(self, gait_type):
        self.gait_type = str(gait_type)

    def set_gait_name(self, gait_name):
        self.gait_name = gait_name

    def set_description(self, description):
        self.description = str(description)

    def set_version(self, version):
        self.version = version

    def set_subgait_name(self, subgait_name):
        self.subgait_name = subgait_name

    def set_duration(self, duration, rescale=False):
        for joint in self.joints:
            # Loop in reverse to avoid out of bounds errors while deleting.
            for setpoint in reversed(joint.setpoints):
                if rescale:
                    setpoint.set_time(duration * setpoint.time / self.duration)
                else:
                    if setpoint.time > duration:
                        joint.setpoints.remove(setpoint)
            joint.interpolated_setpoints = joint.interpolate_setpoints()

            joint.duration = duration

        self.duration = duration
