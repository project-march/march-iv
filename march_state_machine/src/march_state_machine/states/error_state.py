import smach


class ErrorState(smach.State):
    def __init__(self):
        super(ErrorState, self).__init__(outcomes=['succeeded'], input_keys=['sounds'], output_keys=['sounds'])

    def execute(self, userdata):
        if userdata.sounds:
            userdata.sounds.play('error')
        return 'succeeded'
