from PyQt5.QtCore import pyqtSignal, QObject
import rospy

from march_launch.Color import Color


class SoftwareCheck(QObject):
    log_signal = pyqtSignal(str, Color)

    def __init__(self, name, description, timeout=10, manual_confirmation=False):
        QObject.__init__(self, None)
        self.name = name
        self.description = description
        self.manual_confirmation = manual_confirmation
        self.timeout = timeout
        self.passed = False
        self.done = False

    def reset(self):
        self.passed = False
        self.done = False

    def is_passed(self):
        if self.done:
            return self.passed
        return False

    def perform(self):
        raise NotImplementedError('Please implement method "perform()" on the subclass')

    def pass_check(self):
        self.passed = True
        self.done = True

    def fail_check(self):
        self.passed = False
        self.done = True

    def get_key_from_parameter_server(self, key, fail_on_exception=True):
        try:
            value = rospy.get_param(key)
        except KeyError:
            self.log('Could not find key ' + str(key), Color.Error)
            if fail_on_exception:
                self.fail_check()
            return
        return value

    def log(self, msg, level):
        self.log_signal.emit(str(msg), level)
