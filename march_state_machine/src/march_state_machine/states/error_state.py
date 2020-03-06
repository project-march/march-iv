import rospy
import smach


class ErrorState(smach.State):
    def __init__(self):
        super(ErrorState, self).__init__(outcomes=['succeeded'], input_keys=['sounds'], output_keys=['sounds'])

    def execute(self, userdata):
        if userdata.sounds:
            userdata.sounds.play('error')
            # sleep one second to let the sound play
            rospy.sleep(1)
        return 'succeeded'
