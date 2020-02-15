import smach

from march_state_machine.states.transition_state import TransitionState


class TransitionStateMachine(smach.StateMachine):
    def __init__(self, transition_chain, outcomes=None, input_keys=None):
        self._default_input_keys = ['current_gait_name', 'new_gait_name', 'transition_state_name', 'next_state_name']
        self._default_outcomes = ['failed', 'preempted'] + transition_chain
        self._transition_chain = transition_chain

        if outcomes is not None:
            self._default_outcomes = outcomes

        if input_keys is not None:
            self._default_input_keys = input_keys

        super(TransitionStateMachine, self).__init__(outcomes=self._default_outcomes,
                                                     input_keys=self._default_input_keys)

        self.open()
        smach.StateMachine.add('transition',
                               TransitionState(input_keys=self._default_input_keys),
                               transitions={'succeeded': 'failed', 'aborted': 'failed'})
        self.close()

    def _update_once(self):
        """Before entering the transition state set the succeeded transition to the new gait name."""
        self._current_transitions['succeeded'] = self.userdata.new_gait_name
        return super(TransitionStateMachine, self)._update_once()
