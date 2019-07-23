from PyQt5.QtCore import QObject, pyqtSignal
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
        raise NotImplementedError("Please implement method 'perform()' on the subclass")

    def log(self, msg, level):
        self.log_signal.emit(str(msg), level)
