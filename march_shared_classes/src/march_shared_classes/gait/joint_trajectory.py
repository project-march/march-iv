import rospy
from setpoint import Setpoint


class JointTrajectory:
    setpoint_class = Setpoint

    def __init__(self, name, limits, setpoints, duration):
        self.name = name
        self.limits = limits
        self.setpoints = setpoints
        self.duration = duration

    @classmethod
    def from_msg(cls, subgait_msg, joint_name, limits, duration):
        joint_trajectory = subgait_msg.trajectory
        joint_index = joint_trajectory.joint_names.index(joint_name)

        setpoints = []
        for point in joint_trajectory.points:
            time = rospy.Duration(point.time_from_start.secs, point.time_from_start.nsecs).to_sec()
            setpoints.append(cls.setpoint_class(time, point.positions[joint_index],
                                                point.velocities[joint_index]))

        return cls(joint_name,
                   limits,
                   setpoints,
                   duration
                   )

    def get_setpoint(self, index):
        return self.setpoints[index]

    def set_setpoints(self, setpoints):
        self.setpoints = setpoints

    def get_setpoints_unzipped(self):
        time = []
        position = []
        velocity = []

        for i in range(0, len(self.setpoints)):
            time.append(self.setpoints[i].time)
            position.append(self.setpoints[i].position)
            velocity.append(self.setpoints[i].velocity)

        return time, position, velocity
