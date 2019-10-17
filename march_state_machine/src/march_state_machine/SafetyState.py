import smach
import rospy
from march_shared_resources.msg import Error


class SafetyState(smach.State):
    """State which monitors if errors have occurred.

    This state always runs concurrent to the healthy state machine.
    Listens to the topic /march/error and preempts the healthy state machine when an error occurs.
    """

    def __init__(self, outcomes):
        self.listening = True
        self.result = None
        self._subscribers = []
        self._subscribers.append(rospy.Subscriber("march/error", Error, self.error_callback))

        smach.State.__init__(self, outcomes=outcomes)

    def error_callback(self, msg):
        if msg.type == Error.NON_FATAL:
            rospy.logerr("SafetyState has noticed a non fatal error: " + msg.error_message)
            return
        rospy.logerr("SafetyState has noticed an fatal error: " + msg.error_message)
        self.listening = False

    def execute(self, userdata):
        self.listening = True

        # Sleep for a cycle, Simulate rospy.spinOnce() which is not available.
        # We don't want to spin because then we can't exit on a gait instruction.
        rate = rospy.Rate(100)
        while self.listening and not rospy.core.is_shutdown():
            rate.sleep()

        return 'error'

    def unregister_subscribers(self):
        for subscriber in self._subscribers:
            subscriber.unregister()
