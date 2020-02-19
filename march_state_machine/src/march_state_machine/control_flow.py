import rospy

from march_shared_resources.msg import GaitInstruction, GaitInstructionResponse


class ControlFlow:

    def __init__(self):
        self._stopped = False
        self._paused = False
        self._gait = None
        self._transition_index = None

        self._instruction_subscriber = rospy.Subscriber('/march/input_device/instruction',
                                                        GaitInstruction,
                                                        self.callback_input_device_instruction)
        self._instruction_response_publisher = rospy.Publisher('/march/input_device/instruction_response',
                                                               GaitInstructionResponse,
                                                               queue_size=20)

        self._stopped_cb = None
        self._gait_selected_cb = None
        self._gait_transition_cb = None

    def clear_callbacks(self):
        """Clears all currently registered callbacks."""
        self._stopped_cb = None
        self._gait_selected_cb = None
        self._gait_transition_cb = None

    def set_stopped_callback(self, cb):
        """Sets the callback for when stop is pressed.

        :param cb: Callback that will be called with no arguments
        """
        self._stopped_cb = cb

    def set_gait_selected_callback(self, cb):
        """Sets the callback for when a new gait is selected.

        :param cb: Callback that will be called with one string argument gait_name
        """
        self._gait_selected_cb = cb

    def set_gait_transition_callback(self, cb):
        """Sets the callback for when a transition request is selected.

        :param cb: Callback that will be called when a transition is selected
        """
        self._gait_transition_cb = cb

    def get_transition_integer(self):
        """Used to return the transition in the integer-format."""
        if self._transition_index == GaitInstruction.INCREMENT_STEP_SIZE:
            return 1

        if self._transition_index == GaitInstruction.DECREMENT_STEP_SIZE:
            return -1

        return None

    def stop_pressed(self):
        return self._stopped

    def gait_selected(self):
        return self._gait is not None

    def gait_name(self):
        return self._gait

    def is_paused(self):
        return self._paused

    def is_transition(self):
        return self._transition_index

    def reset_gait(self):
        self._gait = None

    def reset_transition(self):
        self._transition_index = None

    def reset_stop(self):
        self._stopped = False

    def gait_accepted(self):
        response = GaitInstructionResponse()
        response.result = GaitInstructionResponse.GAIT_ACCEPTED
        self._instruction_response_publisher.publish(response)
        self.reset_gait()

    def gait_rejected(self):
        response = GaitInstructionResponse()
        response.result = GaitInstructionResponse.GAIT_REJECTED
        self._instruction_response_publisher.publish(response)
        self.reset_gait()

    def gait_finished(self):
        response = GaitInstructionResponse()
        response.result = GaitInstructionResponse.GAIT_FINISHED
        self._instruction_response_publisher.publish(response)
        self.reset_gait()

    def callback_input_device_instruction(self, msg):
        if msg.type == GaitInstruction.STOP:
            self._stopped = True
            if callable(self._stopped_cb):
                self._stopped_cb()

        if msg.type == GaitInstruction.GAIT:
            self._gait = msg.gait_name
            if callable(self._gait_selected_cb):
                self._gait_selected_cb(self._gait)

        if msg.type == GaitInstruction.PAUSE:
            self._paused = True

        if msg.type == GaitInstruction.CONTINUE:
            self._paused = False

        if msg.type == GaitInstruction.INCREMENT_STEP_SIZE:
            self._transition_index = GaitInstruction.INCREMENT_STEP_SIZE
            if callable(self._gait_transition_cb):
                self._gait_transition_cb()

        if msg.type == GaitInstruction.DECREMENT_STEP_SIZE:
            self._transition_index = GaitInstruction.DECREMENT_STEP_SIZE
            if callable(self._gait_transition_cb):
                self._gait_transition_cb()


control_flow = ControlFlow()
