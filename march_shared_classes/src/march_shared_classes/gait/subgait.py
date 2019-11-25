import rospy
from limits import Limits
from joint_trajectory import JointTrajectory
import yaml


class Subgait:
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
    def from_file(cls, robot, file_name):
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
        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version)

    @classmethod
    def from_dict(cls, robot, subgait_dict, gait_name, subgait_name, version):
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

            joint_list.append(cls.joint_class.from_dict(subgait_dict, joint_name, limits, duration))

        return cls(joint_list, duration, subgait_dict['gait_type'], gait_name, subgait_name,
                   version, subgait_dict['description'])

    @staticmethod
    def _get_joint_from_urdf(robot, joint_name):
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

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
