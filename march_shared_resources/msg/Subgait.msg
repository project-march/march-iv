# used in Gait.action
string name
string version
string description
string gait_type

# Filter the interpolated setpoints from the trajectory with the list of user-defined setpoints.
Setpoint[] setpoints

trajectory_msgs/JointTrajectory trajectory
duration duration
Sound[] sounds
