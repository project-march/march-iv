from python_qt_binding.QtWidgets import QMessageBox
import rospy

from .checks.gait_file_directory_check import GaitFileDirectoryCheck
from .checks.git_branch_check import GitBranchCheck
from .checks.slave_count_check import SlaveCountCheck
from .checks.urdf_check import URDFCheck
from .color import Color
from .software_check_thread import SoftwareCheckThread


class CheckRunner:
    def __init__(self, logger=None):
        self.checks = [GitBranchCheck(), GaitFileDirectoryCheck(), URDFCheck(), SlaveCountCheck()]
        for check in self.checks:
            check.log_signal.connect(lambda msg, color: self.log(msg, color))
        self.logger = logger
        self.thread = None

    def run_check_by_name(self, name):
        check = self.get_check(name)
        if check is None:
            self.log('Check with name ' + name + ' does not exist', Color.Error)

        return self.run_check(check)

    def run_check(self, check):
        self.log('--------------------------------------', Color.Info)

        if self.thread is not None:
            self.log('Already running another check', Color.Warning)

        if check is None:
            self.log('Check does not exist', Color.Error)
            return

        self.log('Starting check ' + str(check.name) + ': ' + str(check.description), Color.Info)

        start = rospy.get_rostime()
        self.thread = SoftwareCheckThread(check)
        self.thread.start()

        while not check.done:
            if rospy.get_rostime() < start + rospy.Duration.from_sec(check.timeout):
                rospy.sleep(0.1)

            else:
                self.log('Check ' + str(check.name) + ' timed out after ' + str(check.timeout) + 's', Color.Error)
                self.thread.exit()
                self.thread = None

                check.reset()
                return False

        self.thread.wait()
        self.thread = None
        result = check.passed
        if result and check.manual_confirmation:
            result = self.validate_manually()
        check.reset()

        if result:
            self.log('Check ' + str(check.name) + ' was succesful!', Color.Debug)
        else:
            self.log('Check ' + str(check.name) + ' has failed', Color.Error)

        return result

    def get_check(self, name):
        for check in self.checks:
            if check.name == name:
                return check
        return None

    def log(self, msg, level):
        if self.logger is not None:
            self.logger(msg, level)

    @staticmethod
    def validate_manually():
        reply = QMessageBox.question(None, 'Message', 'Did the test pass?', QMessageBox.Yes, QMessageBox.No)
        return reply == QMessageBox.Yes
