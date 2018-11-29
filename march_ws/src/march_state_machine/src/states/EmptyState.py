import smach

import time


class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        time.sleep(1)
        return 'succeeded'
