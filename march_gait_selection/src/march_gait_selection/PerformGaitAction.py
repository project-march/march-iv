import actionlib
import rospy

from march_shared_classes.exceptions.general_exceptions import MsgTypeError
from march_shared_classes.exceptions.gait_exceptions import *
from march_shared_resources import msg
from march_shared_resources.msg import GaitAction, GaitGoal, GaitNameAction

SERVER_TIMEOUT = 5
RESPONSE_TIMEOUT = 1


class PerformGaitAction(object):
    def __init__(self, gait_selection):
        self.gait_selection = gait_selection
        self.action_server = actionlib.SimpleActionServer('/march/gait/perform', GaitNameAction,
                                                          execute_cb=self.target_gait_callback,
                                                          auto_start=False)
        self.action_server.start()
        self.schedule_gait_client = actionlib.SimpleActionClient('/march/gait/schedule', GaitAction)

        while not rospy.is_shutdown() and not self.schedule_gait_client.wait_for_server(rospy.Duration(SERVER_TIMEOUT)):
            rospy.logdebug('Waiting for /march/gait/schedule to come up')

    def target_gait_callback(self, subgait_goal_msg):
        """Set a new target subgait over the action server march/gait/schedule."""
        rospy.logdebug('Trying to schedule subgait {gn} {sn}'
                       .format(gn=subgait_goal_msg.name, sn=subgait_goal_msg.subgait_name))

        gait = self.gait_selection[subgait_goal_msg.name]
        if gait:
            subgait = gait[subgait_goal_msg.subgait_name]
            if subgait:
                trajectory_state = self.schedule_gait(subgait_goal_msg.name, subgait.to_subgait_msg())
                if trajectory_state == actionlib.GoalStatus.SUCCEEDED:
                    self.action_server.set_succeeded(trajectory_state)
                else:
                    self.action_server.set_aborted(trajectory_state)
                return True

        rospy.logwarn('Gait {gn} {sn} does not exist in parsed gaits'
                      .format(gn=subgait_goal_msg.name, sn=subgait_goal_msg.subgait_name))

        self.action_server.set_aborted('Gait {gn} does not exist in parsed gaits'
                                       .format(gn=subgait_goal_msg.name))
        return False

    def add_transition_trajectory(self, current_gait, new_gait, last_subgait):
        current_gait_object = self.gait_selection[current_gait]
        new_gait_object = self.gait_selection[new_gait]

        if current_gait_object is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=current_gait))

        if new_gait_object is None:
            raise GaitError(msg='{gn} not found in parsed gait names from gait selection'
                            .format(gn=new_gait))

        if current_gait_object.to_subgaits_names != new_gait_object.to_subgaits_names:
            raise GaitError(msg='To_subgait list do not match between gait: {cg} and gait: {ng}'
                            .format(cg=current_gait, ng=new_gait))

        if current_gait_object.from_subgaits_names != new_gait_object.from_subgaits_names:
            raise GaitError(msg='From_subgait list do not match between gait: {cg} and gait: {ng}'
                            .format(cg=current_gait, ng=new_gait))

        new_subgait_index = current_gait_object.to_subgaits_names.index(last_subgait)

        old_subgait_object = current_gait_object[last_subgait]
        new_subgait_object = new_gait_object.to_subgaits_names[new_subgait_index]

        if old_subgait_object is None:
            raise SubgaitNameNotFound(msg='Subgait {sg} not found in to_subgaits of gait {gn}'
                                      .format(sg=last_subgait, gn=current_gait))

        if new_subgait_object is None:
            raise SubgaitNameNotFound(msg='Subgait {sg} not found in from_subgaits of gait {gn}'
                                      .format(sg=last_subgait, gn=current_gait))

        for joint_new_gait in new_subgait_object.joints:
            old_subgait_joint = old_subgait_object[joint_new_gait.name]
            joint_last_setpoint = old_subgait_joint.setpoints[-1]
            old_subgait_joint.setpoints.insert(0, joint_last_setpoint)

    def schedule_gait(self, gait_name, subgait):
        """Construct the goal message and send."""
        if type(subgait) != msg.Subgait:
            raise MsgTypeError(msg='Given subgait is not of type msg.Subgait')

        gait_action_goal = GaitGoal()
        gait_action_goal.name = gait_name
        gait_action_goal.current_subgait = subgait

        self.schedule_gait_client.send_goal(gait_action_goal)
        self.schedule_gait_client.wait_for_result(timeout=subgait.duration + rospy.Duration(RESPONSE_TIMEOUT))

        return self.schedule_gait_client.get_state()
