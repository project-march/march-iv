import rospy
from march_shared_resources.msg import GaitInstruction
from std_msgs.msg import Bool


class ControlFlow:

    def __init__(self):
        self._stopped = False
        self._paused = False
        self._gait = None
        self._input_device_instruction_subscriber = rospy.Subscriber('/march/input_device/instruction', GaitInstruction,
                                                                     self.callback_input_device_instruction)
        self._input_device_instruction_response_publisher = rospy.Publisher('/march/input_device/instruction_response',
                                                                            Bool,
                                                                            queue_size=20)

    def stop_pressed(self):
        return self._stopped

    def gait_selected(self):
        return self._gait is not None

    def gait_name(self):
        return self._gait

    def is_paused(self):
        return self._paused

    def reset_gait(self):
        self._gait = None

    def reset_stop(self):
        self._stopped = False

    def gait_accepted(self):
        self._input_device_instruction_response_publisher.publish(True)
        self.reset_gait()

    def gait_rejected(self):
        self._input_device_instruction_response_publisher.publish(False)
        self.reset_gait()

    def callback_input_device_instruction(self, msg):
        if msg.type == GaitInstruction.STOP:
            self._stopped = True
        if msg.type == GaitInstruction.GAIT:
            self._gait = msg.gait_name
        if msg.type == GaitInstruction.PAUSE:
            self._paused = True
        if msg.type == GaitInstruction.CONTINUE:
            self._paused = False


control_flow = ControlFlow()
