import ast
import os
import sys

import rospy
import rospkg
from march_shared_resources.srv import StringTrigger, Trigger

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QPlainTextEdit

from CheckRunner import CheckRunner
from Color import Color


class SoftwareCheckPlugin(Plugin):

    def __init__(self, context):
        super(SoftwareCheckPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SoftwareCheckPlugin')
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('march_launch'), 'resource', 'software_check.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SoftwareCheck')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.log("Welcome to the Software Check.", Color.Debug)
        self.log("Select the software checks you want to perform.", Color.Info)

        self.check_runner = CheckRunner(self.log)

        self._widget.Checks.findChild(QPushButton, "GaitFileDirectory").clicked.connect(lambda: self.run_test("GaitFileDirectory"))
        self._widget.Checks.findChild(QPushButton, "URDF").clicked.connect(lambda: self.run_test("URDF"))
        self._widget.Checks.findChild(QPushButton, "SlaveCount").clicked.connect(lambda: self.run_test("SlaveCount"))

    def log(self, msg, level):
        self._widget.findChild(QPlainTextEdit, "Log").appendHtml(
            "<p style='color:" + str(level.value) + "'>" + str(msg) + "</p>")
        scrollbar = self._widget.findChild(QPlainTextEdit, "Log").verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def run_test(self, name):
        result = self.check_runner.run_check_by_name(name)
        self.update_test_button(name, result)

    def update_test_button(self, name, result):
        button = self._widget.Checks.findChild(QPushButton, name)
        if result:
            button.setStyleSheet("background-color: " + str(Color.Check_Passed.value))
        else:
            button.setStyleSheet("background-color: " + str(Color.Check_Failed.value))
