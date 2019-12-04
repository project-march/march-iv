import time

import smach


class EmptyState(smach.State):
    """Empty State which succeeds after a timeout.

    Can be used as a placeholder to design the whole state machine without functionality.
    """

    def __init__(self, sleep_time=0.1):
        self.sleep_time = sleep_time
        super(EmptyState, self).__init__(outcomes=['succeeded'])

    """Sleep so the statemachine stays in this state longer."""
    def execute(self, userdata):
        time.sleep(self.sleep_time)
        return 'succeeded'
