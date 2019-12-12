import rospy

from .setpoint import Setpoint


class JointTrajectory(object):
    setpoint_class = Setpoint

    def __init__(self, name, limits, setpoints, duration):
        self.name = name
        self.limits = limits
        self.setpoints = setpoints
        self.duration = duration

    @classmethod
    def from_dict(cls, subgait_dict, joint_name, limits, duration, *args):
        joint_trajectory = subgait_dict['trajectory']
        joint_index = joint_trajectory['joint_names'].index(joint_name)

        setpoints = []
        for point in joint_trajectory['points']:
            time = rospy.Duration(point['time_from_start']['secs'], point['time_from_start']['nsecs']).to_sec()
            setpoints.append(cls.setpoint_class(time, point['positions'][joint_index],
                                                point['velocities'][joint_index]))

        return cls(joint_name,
                   limits,
                   setpoints,
                   duration,
                   *args
                   )

    def get_setpoints_unzipped(self):
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)
