import smach_ros
import rospy
from actionlib_msgs.msg import GoalStatus


def action_result_cb(userdata, status, result):

    if status == GoalStatus.SUCCEEDED:
        return 'succeeded'
    else:
        return 'aborted'


class FeedbackActionState(smach_ros.SimpleActionState):
    def __init__(self, action_topic, action, goal, outcomes, input_keys=[], output_keys=[]):
        self._subscribers = []
        smach_ros.SimpleActionState.__init__(self, action_topic, action, goal,
                                             result_cb=action_result_cb,
                                             outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

    def _goal_feedback_cb(self, feedback):
        rospy.logdebug(feedback)

    # def unregister_subscribers(self):
    #     for subscriber in self._subscribers:
    #         subscriber.unregister()
