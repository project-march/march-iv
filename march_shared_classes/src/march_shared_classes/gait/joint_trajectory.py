import rospy

from .setpoint import Setpoint


class JointTrajectory(object):
    """Base class for joint trajectory of a gait."""

    setpoint_class = Setpoint

    def __init__(self, name, limits, setpoints, duration, *args):
        self.name = name
        self.limits = limits
        self.setpoints = setpoints
        self.duration = duration

    @classmethod
    def from_dict(cls, subgait_dict, joint_name, limits, duration, *args):
        """Create class of JointTrajectory with filled attributes.

        :param subgait_dict:
            The dictionary extracted from the yaml file
        :param joint_name:
            The name of the joint corresponding to this specific object
        :param limits:
            Defined soft limits of the urdf file
        :param duration:
            The timestamps of the subgait file
        """
        joint_trajectory = subgait_dict['trajectory']
        joint_index = joint_trajectory['joint_names'].index(joint_name)

        setpoints = []
        for point in joint_trajectory['points']:
            time = rospy.Duration(point['time_from_start']['secs'], point['time_from_start']['nsecs']).to_sec()
            setpoints.append(cls.setpoint_class(time, point['positions'][joint_index],
                                                point['velocities'][joint_index]))

        return cls(joint_name, limits, setpoints, duration, *args)

    def get_setpoints_unzipped(self):
        """Get all the listed attributes of the setpoints."""
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def validate_joint_transition(self, joint):
        """Validate the ending and starting of this joint to a given joint.

        :param joint:
            the joint of the next subgait (not the previous one)

        :returns:
            True if ending and starting point are identical else False
        """
        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoint[0]

        if from_setpoint.velocity == to_setpoint.velocity and from_setpoint.position == to_setpoint.position:
            return True

        return False

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)
