import numpy as np
import rospy
from scipy.interpolate import BPoly

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
        if not self._validate_boundary_points():
            return False

        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoints[0]

        if from_setpoint.velocity == to_setpoint.velocity and from_setpoint.position == to_setpoint.position:
            return True

        return False

    def _validate_boundary_points(self):
        """Validate the starting and ending of this joint are at t = 0 and t = duration, or that their speed is zero.

        :returns:
            False if the starting/ending point is (not at 0/duration) and (has nonzero speed), True otherwise
        """
        return (self.setpoints[0].time == 0 or self.setpoints[0].velocity == 0) and \
               (self.setpoints[-1].time == round(self.duration, Setpoint.digits) or self.setpoints[-1].velocity == 0)

    def interpolate_setpoints(self):
        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        # We do a cubic spline here, just like the ros joint_trajectory_action_controller,
        # see https://wiki.ros.org/robot_mechanism_controllers/JointTrajectoryActionController
        position = BPoly.from_derivatives(time, yi)
        velocity = position.derivative()
        indices = np.linspace(0, self.duration, self.duration * 100)
        return [indices, position(indices), velocity(indices)]

    def get_interpolated_setpoint(self, time):
        # If we have a setpoint this exact time there is no need to interpolate.
        for setpoint in self.setpoints:
            if setpoint.time == time:
                return setpoint

        interpolated_setpoints = self.interpolate_setpoints()
        for i in range(0, len(interpolated_setpoints[0])):
            if interpolated_setpoints[0][i] > time:
                position = interpolated_setpoints[1][i - 1]
                velocity = interpolated_setpoints[2][i - 1]
                return self.setpoint_class(time, position, velocity)
        rospy.logerr('Could not interpolate setpoint at time {0}'.format(time))
        return self.setpoint_class(0, 0, 0)

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)
