import os
import sys

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QRadioButton
from python_qt_binding.QtWidgets import QCheckBox
from march_rqt_launch_menu.run_configuration_enum import run_configuration_enum


class LaunchMenuPlugin(Plugin):

    def __init__(self, context):
        super(LaunchMenuPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LaunchMenuPlugin')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_launch_menu'), 'resource', 'launch_menu.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Launch Menu')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        run_configurations = self._widget.RunConfigurationGroup.children()
        for run_configuration in run_configurations:
            if type(run_configuration) == QRadioButton:
                run_configuration.clicked.connect(lambda: self.run_configuration_changed())

        self._widget.FinalButtons.accepted.connect(self.accept_settings)
        self._widget.FinalButtons.rejected.connect(self.reject_settings)

    def accept_settings(self):
        dictionary = {}

        # Loop through all groupboxes and get the active radiobutton if they have one.
        groupboxes = self._widget.findChildren(QGroupBox)
        for groupbox in groupboxes:
            if groupbox.isEnabled() and self.has_radio_buttons(groupbox):
                topic = groupbox.statusTip()
                setting_value = self.get_active_radio_button(groupbox)
                dictionary[topic] = setting_value

        # Parse all checkboxes and add their boolean value to the dictionary.
        checkboxes = self._widget.findChildren(QCheckBox)
        for checkbox in checkboxes:
            topic = str(checkbox.statusTip())
            if not topic:
                rospy.logwarn("Empty key found at checkbox with text " +
                              checkbox.text() + ". Please confirm you have added a topic as statusTip.")
            # Only add enabled checkboxes to avoid confusion when looking at the parameter list.
            if checkbox.isEnabled():
                dictionary[topic] = checkbox.isChecked()

        self.publish_to_parameter_server(dictionary)
        sys.exit(0)

    def has_radio_buttons(self, groupbox):
        # Check if the list is empty.
        return not groupbox.findChildren(QCheckBox)

    def get_active_radio_button(self, groupbox):
        children = groupbox.children()
        for run_configuration in children:
            if type(run_configuration) == QRadioButton and run_configuration.isChecked():
                return run_configuration.text().lower().replace(" ", "")

    def get_run_configuration(self):
        return self.get_active_radio_button(self._widget.RunConfigurationGroup)

    @staticmethod
    def publish_to_parameter_server(dictionary):
        rospy.loginfo("Uploading the following launch_settings to the parameter server: " + str(dictionary))
        for key, value in dictionary.iteritems():
            if key[0] == '~':
                rospy.set_param(key[1:], value)
            else:
                rospy.set_param("march/launch_settings" + key, value)
        rospy.set_param("march/launch_settings/success", True)

    @staticmethod
    def reject_settings():
        rospy.logfatal("Pressed cancel on the launch settings menu.")
        rospy.set_param("march/launch_settings/success", False)
        sys.exit(1)

    """Callback for when the run configuration setting changes."""
    def run_configuration_changed(self):
        self.disable_all_run_configurations()
        enabled_configuration = self.get_run_configuration()

        if enabled_configuration == run_configuration_enum.exoskeleton.name:
            self._widget.ExoskeletonGroup.setEnabled(True)
        elif enabled_configuration == run_configuration_enum.simulation.name:
            self._widget.SimulationGroup.setEnabled(True)
        elif run_configuration_enum.has_value(enabled_configuration):
            rospy.logwarn("Run configuration '" + str(enabled_configuration) +
                          "' exists but has no QGroupBox associated with it.")
        else:
            rospy.logfatal("Run configuration '" + str(enabled_configuration) + "' does not exist.")
            sys.exit(1)

    """Disable all Widgets in the RunConfigurationLayout."""
    def disable_all_run_configurations(self):
        for widget in self.get_layout_widgets(self._widget.RunConfigurationLayout):
            widget.setEnabled(False)

    """Return all widgets found in the requested layout."""
    def get_layout_widgets(self, layout):
        return (layout.itemAt(i).widget() for i in range(layout.count()))

# def trigger_configuration(self):
# Comment in to signal that the plugin has a way to configure
# This will enable a setting button (gear icon) in each dock widget title bar
# Usually used to open a modal configuration dialog
