import smach
import time


class EmptyState(smach.State):
    """Empty State which succeeds after a timeout.
    Can be used as a placeholder to design the whole statemachine without functionality.
    """

    def __init__(self, sleep_time=0):
        self.sleep_time = sleep_time
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    """Sleep so the statemachine stays in this state longer."""
    def execute(self, userdata):
        time.sleep(self.sleep_time)
        return 'succeeded'
