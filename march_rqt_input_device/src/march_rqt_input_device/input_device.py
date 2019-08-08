import os
import rospy
import rospkg
import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QWidget
from march_shared_resources.msg import Error
from march_shared_resources.msg import GaitInstruction

from march_rqt_input_device.MarchButton import MarchButton
from march_rqt_input_device.LayoutBuilder import LayoutBuilder

from march_rqt_input_device.PublishAliveThread import PublishAliveThread


class InputDevicePlugin(Plugin):

    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('InputDevicePlugin')

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
        # Get path to UI file which should be in
        # the "resource" folder of this package
        ui_file = os.path.join(
            rospkg.RosPack().get_path('march_rqt_input_device'), 'resource',
            'input_device.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Input Device')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (
                    ' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Create buttons here
        home_sit_button = MarchButton(name="home_sit", image="/home_sit.png",
                                      callback=lambda: self.publish_gait(
                                          "home_sit"))
        home_stand_button = MarchButton(name="home_stand",
                                        image="/home_stand.png",
                                        callback=lambda: self.publish_gait(
                                            "home_stand"))
        gait_sit_button = MarchButton(name="gait_sit", image="/gait_sit.png",
                                      callback=lambda: self.publish_gait(
                                          "gait_sit"))
        gait_walk_button = MarchButton(name="gait_walk",
                                       image="/gait_walk.png",
                                       callback=lambda: self.publish_gait(
                                           "gait_walk"))
        gait_single_step_small_button = MarchButton(name="gait_single_step_small",
                                                    text="Single step small",
                                                    callback=lambda: self.publish_gait(
                                                        "gait_single_step_small"))
        gait_single_step_normal_button = MarchButton(name="gait_single_step_normal",
                                                     text="Single step normal",
                                                     callback=lambda: self.publish_gait(
                                                         "gait_single_step_normal"))
        gait_side_step_left_button = MarchButton(name="gait_side_step_left",
                                                 text="Side step left",
                                                 callback=lambda: self.publish_gait(
                                                     "gait_side_step_left"))
        gait_side_step_right_button = MarchButton(name="gait_side_step_right",
                                                  text="Side step right",
                                                  callback=lambda: self.publish_gait(
                                                      "gait_side_step_right"))
        gait_stand_button = MarchButton(name="gait_stand",
                                        image="/gait_stand.png",
                                        callback=lambda: self.publish_gait(
                                            "gait_stand"))
        gait_sofa_stand_button = MarchButton(name="gait_sofa_stand",
                                             text="Sofa stand",
                                             callback=lambda: self.publish_gait(
                                                 "gait_sofa_stand"))
        gait_sofa_sit_button = MarchButton(name="gait_sofa_sit",
                                           text="Sofa sit",
                                           callback=lambda: self.publish_gait(
                                               "gait_sofa_sit"))
        gait_stairs_up_button = MarchButton(name="gait_stairs_up",
                                            text="Stairs up",
                                            callback=lambda: self.publish_gait(
                                                "gait_stairs_up"))
        gait_stairs_down_button = MarchButton(name="gait_stairs_down",
                                              text="Stairs down",
                                              callback=lambda: self.publish_gait(
                                                  "gait_stairs_down"))
        gait_stairs_down_final_step_button = MarchButton(name="gait_stairs_down_final_step",
                                                         text="Stairs down final step",
                                                         callback=lambda: self.publish_gait(
                                                             "gait_stairs_down_final_step"))

        stop_button = MarchButton(name="gait_stop", image="/stop.png",
                                  callback=lambda: self.publish_stop())
        pause_button = MarchButton(name="gait_pause", text="Pause",
                                   callback=lambda: self.publish_pause())
        continue_button = MarchButton(name="gait_continue", text="Continue",
                                      callback=lambda: self.publish_continue())

        error_button = MarchButton(name="error", image="/error.png",
                                   callback=lambda: self.publish_error())

        # The button layout.
        # Position in the array determines position on screen.
        march_button_layout = [
            [home_sit_button, home_stand_button, gait_walk_button],
            [gait_sit_button, gait_stand_button, gait_single_step_normal_button],
            [gait_sofa_sit_button, gait_sofa_stand_button, gait_single_step_small_button],
            [gait_side_step_left_button, gait_side_step_right_button, gait_stairs_up_button],
            [gait_stairs_down_button, gait_stairs_down_final_step_button, stop_button],
            [pause_button, continue_button, error_button],
        ]

        # Create the qt_layout from the button layout.
        layout_builder = LayoutBuilder(march_button_layout)
        qt_layout = layout_builder.build()
        # Apply the qt_layout to the top level widget.
        self._widget.frame.setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(15)
        self._widget.frame.adjustSize()

        # ROS publishers.
        # It is important that you unregister them in the self.shutdown method.
        self.instruction_gait_pub = rospy.Publisher(
            'march/input_device/instruction', GaitInstruction, queue_size=10)

        self.error_pub = rospy.Publisher('march/error', Error, queue_size=10)

        if rospy.get_param("ping_safety_node", "true"):
            self.alive_thread = PublishAliveThread()
            self.alive_thread.start()

    def shutdown_plugin(self):
        self.alive_thread.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def publish_gait(self, string):
        rospy.logdebug("Mock Input Device published gait: " + string)
        self.instruction_gait_pub.publish(GaitInstruction(GaitInstruction.GAIT, string))

    def publish_stop(self):
        rospy.logdebug("Mock Input Device published stop")
        self.instruction_gait_pub.publish(GaitInstruction(GaitInstruction.STOP, ""))

    def publish_continue(self):
        rospy.logdebug("Mock Input Device published continue")
        self.instruction_gait_pub.publish(GaitInstruction(GaitInstruction.CONTINUE, ""))

    def publish_pause(self):
        rospy.logdebug("Mock Input Device published pause")
        self.instruction_gait_pub.publish(GaitInstruction(GaitInstruction.PAUSE, ""))

    def publish_error(self):
        rospy.logdebug("Mock Input Device published error")
        self.error_pub.publish(Error("Fake error thrown by the develop input device.", Error.FATAL))

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (the gear icon)
    # in each dock widget title bar
    # Usually used to open a modal configuration dialog
