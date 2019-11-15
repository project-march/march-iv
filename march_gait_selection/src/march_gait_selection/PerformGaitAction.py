import rospy
import actionlib

from march_shared_resources.msg import GaitNameAction, GaitAction, GaitGoal


class PerformGaitAction(object):

    def __init__(self, gait_selection):
        self.gait_selection = gait_selection
        self.action_server = actionlib.SimpleActionServer('/march/gait/perform', GaitNameAction,
                                                          execute_cb=self.target_gait_callback,
                                                          auto_start=False)
        self.action_server.start()

        self.schedule_gait_client = actionlib.SimpleActionClient('/march/gait/schedule', GaitAction)
        while not rospy.is_shutdown() and not self.schedule_gait_client.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo('Waiting for /march/gait/schedule to come up')

    def target_gait_callback(self, goal):
        rospy.logdebug('Trying to schedule subgait %s/%s', goal.name, goal.subgait_name)
        if not self.gait_selection.validate_gait_by_name(goal.name):
            rospy.logerr('Gait %s is invalid', goal.name)
            self.action_server.set_aborted('Gait ' + str(goal.name) + 'is invalid')
            return False
        subgait = self.gait_selection.get_subgait(goal.name, goal.subgait_name)
        trajectory_state = self.schedule_gait(goal.name, subgait)

        if trajectory_state == actionlib.GoalStatus.SUCCEEDED:
            self.action_server.set_succeeded(trajectory_state)
        else:
            self.action_server.set_aborted(trajectory_state)

    def schedule_gait(self, gait_name, subgait):
        gait_action_goal = GaitGoal()
        gait_action_goal.name = gait_name
        gait_action_goal.current_subgait = subgait

        self.schedule_gait_client.send_goal(gait_action_goal)

        self.schedule_gait_client.wait_for_result(timeout=subgait.duration + rospy.Duration(1))
        return self.schedule_gait_client.get_state()
