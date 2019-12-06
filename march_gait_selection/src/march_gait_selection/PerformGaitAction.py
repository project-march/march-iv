import actionlib
import rospy

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
            rospy.loginfo('Waiting for /march/gait/schedule to come up')

    def target_gait_callback(self, goal):
        """Set a new target subgait over the action server."""
        rospy.logdebug('Trying to schedule subgait {gn} {sn}'.format(gn=goal.name, sn=goal.subgait_name))

        gait = self.gait_selection[goal.name]
        if gait:
            subgait = gait[goal.subgait_name]
            if subgait:
                trajectory_state = self.schedule_gait(goal.name, subgait)
                if trajectory_state == actionlib.GoalStatus.SUCCEEDED:
                    self.action_server.set_succeeded(trajectory_state)
                else:
                    self.action_server.set_aborted(trajectory_state)

        rospy.logdebug('Gait {gn} {sn} does not exist in parsed gaits'.format(gn=goal.name, sn=goal.subgait_name))
        self.action_server.set_aborted('Gait {gn} does not exist in parsed gaits'.format(gn=goal.name))
        return False

    def schedule_gait(self, gait_name, subgait):
        """Construct the goal message and send."""
        gait_action_goal = GaitGoal()
        gait_action_goal.name = gait_name
        gait_action_goal.current_subgait = subgait

        self.schedule_gait_client.send_goal(gait_action_goal)
        self.schedule_gait_client.wait_for_result(timeout=subgait.duration + rospy.Duration(RESPONSE_TIMEOUT))

        return self.schedule_gait_client.get_state()
