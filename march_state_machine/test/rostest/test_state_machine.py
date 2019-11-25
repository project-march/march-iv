#!/usr/bin/env python
import unittest

import rospy
from smach import InvalidTransitionError, InvalidStateError

import march_state_machine.StateMachine as StateMachine

PKG = 'march_state_machine'


class TestStateMachine(unittest.TestCase):

    """Check the entire state machine for consistency.
    This asserts that all transition targets are states that are in the
    state machine. If this fails, it raises an L{InvalidTransitionError}
    with relevant information.

    Validate full state specification (label, state, and transitions).
    This checks to make sure the required variables are in the state spec,
    as well as verifies that all outcomes referenced in the transitions
    are registered as valid outcomes in the state object. If a state
    specification fails validation, a L{smach.InvalidStateError} is
    thrown.
    """
    def test_state_machine_consistency(self):
        rospy.init_node("test_state_machine", anonymous=True, disable_signals=True)

        try:
            state_machine = StateMachine.create_sm()
            state_machine.check_consistency()
        except InvalidStateError:
            self.fail("State machine has invalid states. Launch it independently for more information.")
        except InvalidTransitionError:
            self.fail("State machine has invalid transitions. Launch it independently for more information.")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_state_machine', TestStateMachine)
