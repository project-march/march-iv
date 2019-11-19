import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from march_shared_resources.msg import Setpoint


class Subgait:
    def __init__(self, joints, duration, gait_type='walk_like',
                 name='Walk', subgait='right_open', version='First try', description='Just a simple gait'):
        # Set gait_type to walk_like if an old file with no gait_type is opened
        if gait_type == '':
            gait_type = 'walk_like'

        self.joints = joints
        self.gait_type = gait_type
        self.name = name
        self.subgait = subgait
        self.version = version
        self.description = str(description)
        self.duration = duration

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
        rospy.logerr('Joint with name ' + name + ' does not exist in gait ' + self.name)
        return None

    def set_gait_type(self, gait_type):
        self.gait_type = str(gait_type)

    def set_name(self, name):
        self.name = name

    def set_description(self, description):
        self.description = str(description)

    def set_version(self, version):
        self.version = version

    def set_subgait(self, subgait):
        self.subgait = subgait

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
