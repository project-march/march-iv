import subprocess

from .software_check import SoftwareCheck


class LaunchCheck(SoftwareCheck):

    def __init__(self, name, description, package, launch_file, timeout=10000, manual_confirmation=False):
        SoftwareCheck.__init__(self, name, description, timeout, manual_confirmation)
        self.package = package
        self.file = launch_file
        self.launch_process = None

    def launch(self):
        cmd = 'roslaunch ' + self.package + ' ' + self.file
        self.launch_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash')

    def perform(self):
        raise NotImplementedError('Please implement method "perform()" on the subclass')

    def stop_launch_process(self):
        self.launch_process.kill()
