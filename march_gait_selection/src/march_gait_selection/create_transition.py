from march_shared_classes.exceptions.gait_exceptions import GaitError, SubgaitNameNotFound
from march_shared_classes.gait.subgait import Subgait


class TransitionSubgait:
    def __init__(self):
        pass

    @staticmethod
    def create_transition_subgait(gait_selection, current_gait_name, new_gait_name, last_subgait_name,
                                  transition_time=500000000):
        """ """
        current_gait_object = gait_selection[current_gait_name]
        new_gait_object = gait_selection[new_gait_name]

        if current_gait_object is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=current_gait_name))

        if new_gait_object is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=new_gait_name))

        if current_gait_object.to_subgaits_names != new_gait_object.to_subgaits_names:
            raise GaitError(msg='To_subgait list do not match between gait: {cg} and gait: {ng}'
                            .format(cg=current_gait_name, ng=new_gait_name))

        if current_gait_object.from_subgaits_names != new_gait_object.from_subgaits_names:
            raise GaitError(msg='From_subgait list do not match between gait: {cg} and gait: {ng}'
                            .format(cg=current_gait_name, ng=new_gait_name))

        old_subgait, new_subgait = TransitionSubgait.get_start_end_subgaits(current_gait_object,
                                                                            new_gait_object,
                                                                            last_subgait_name)

        robot = gait_selection.robot
        gait_name = 'transition'
        subgait_name = 'transition_subgait'
        version = 'default'

        duration = TransitionSubgait.transition_duration(transition_time)
        trajectory = TransitionSubgait.transition_trajectory(old_subgait, new_subgait, transition_time)

        subgait_dict = {'trajectory': trajectory,
                        'duration': duration}

        return Subgait.from_dict(robot, subgait_dict, gait_name, subgait_name, version)

    @staticmethod
    def transition_duration(transition_time):
        """ """
        return {'secs': 0, 'nsecs': transition_time}

    @staticmethod
    def transition_trajectory(old_subgait, new_subgait, interval_time):
        """ """
        trajectory = {}

        joints = []
        positions = [[], []]
        velocities = [[], []]
        for joint_old_gait in old_subgait.joints:
            joint_new_gait = new_subgait.get_joint(joint_old_gait.name)

            old_setpoint = joint_old_gait.setpoints[-1]
            new_setpoint = joint_new_gait.setpoints[0]

            positions[0].append(old_setpoint.position)
            positions[1].append(new_setpoint.position)

            velocities[0].append(old_setpoint.velocity)
            velocities[1].append(new_setpoint.velocity)

            joints.append(joint_old_gait.name)

        points = []
        transition_time = 0
        for position_list, velocity_list in zip(positions, velocities):
            point = {'positions': position_list,
                     'velocities': velocity_list,
                     'time_from_start': {'secs': 0, 'nsecs': transition_time},
                     }
            transition_time += interval_time
            points.append(point)

        trajectory['joint_names'] = joints
        trajectory['points'] = points

        return trajectory

    @staticmethod
    def get_start_end_subgaits(old_gait, new_gait, last_subgait_name):
        """ """
        new_subgait_index = new_gait.from_subgaits_names.index(last_subgait_name)
        new_subgait_name = new_gait.to_subgaits_names[new_subgait_index]

        old_subgait = old_gait[last_subgait_name]
        new_subgait = new_gait[new_subgait_name]

        if old_subgait is None:
            raise SubgaitNameNotFound(subgait_name=last_subgait_name)

        if new_subgait is None:
            raise SubgaitNameNotFound(subgait_name=new_subgait_name)

        return old_subgait, new_subgait
