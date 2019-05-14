import ast
import math
import os

import rospy
import rospkg
from enum import Enum
from march_shared_resources.srv import StringTrigger, Trigger

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QLayout
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QPlainTextEdit
from python_qt_binding.QtWidgets import QCheckBox

import networkx as nx
import pyqtgraph

class Color(Enum):
    Debug = "#009100"
    Info = "#000000"
    Warning = "#b27300"
    Error = "#FF0000"
    Fatal = "#FF0000"

class GaitSelectionPlugin(Plugin):

    def __init__(self, context):
        super(GaitSelectionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GaitSelectionPlugin')
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

        # Connect to services
        rospy.wait_for_service('/march/gait_selection/get_version_map', 3)
        self.get_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)
        rospy.wait_for_service('/march/gait_selection/get_directory_structure', 3)
        self.get_directory_structure = rospy.ServiceProxy('/march/gait_selection/get_directory_structure', Trigger)
        rospy.wait_for_service('/march/gait_selection/set_version_map', 3)
        self.set_version_map = rospy.ServiceProxy('/march/gait_selection/set_version_map', StringTrigger)
        rospy.wait_for_service('/march/gait_selection/update_default_versions', 3)
        self.update_default_versions = rospy.ServiceProxy('/march/gait_selection/update_default_versions', Trigger)

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_selection'), 'resource', 'gait_selection.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Gait Selection')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.refresh()

        self._widget.findChild(QPushButton, "Refresh").clicked.connect(lambda: self.refresh(True))
        self._widget.findChild(QPushButton, "Apply").clicked.connect(
            lambda: [
                self.set_gait_selection_map(True),
                self.refresh()
            ])
        self._widget.findChild(QPushButton, "SaveDefault").clicked.connect(
            lambda: [
                self.set_gait_selection_map(),
                self.update_defaults(True),
                self.refresh()
            ])

        self.log("Welcome to the Gait Selection.", Color.Debug)
        self.log("Select the versions of subgaits you want to select and press Apply.", Color.Info)
        self.log("Save as default persists between launches", Color.Info)
        self.log("Any warnings or errors will be displayed in this log.", Color.Warning)
        self.log("--------------------------------------", Color.Info)

    def log(self, msg, level):
        self._widget.findChild(QPlainTextEdit, "Log").appendHtml("<p style='color:" + str(level.value) + "'>" + msg + "</p>")
        scrollbar = self._widget.findChild(QPlainTextEdit, "Log").verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def refresh(self, notify=False):
        if notify:
            self.log("Refreshing gait directory", Color.Debug)
        rospy.logdebug("Refreshing ui with %s", str(self.get_directory_structure().message))

        try:
            gait_version_map = ast.literal_eval(self.get_version_map().message)
        except ValueError:
            self.log("Gait selection map is not valid" + str(self.get_version_map().message), Color.Error)
            rospy.logerr("Gait selection map is not valid" + str(self.get_version_map().message))
            return
        try:
            gait_directory_structure = ast.literal_eval(self.get_directory_structure().message)
        except ValueError:
            self.log("Gait directory structure is not valid " + str(self.get_directory_structure().message), Color.Error)
            rospy.logerr("Gait directory structure is not valid " + str(self.get_directory_structure().message))
            return

        gaits = self._widget.Gaits.findChildren(QGroupBox, "Gait")
        for gait in gaits:
            gait.deleteLater()

        self.load_ui(gait_directory_structure, gait_version_map)
        self._widget.layout().setSizeConstraint(QLayout.SetFixedSize)

    def load_ui(self, gait_directory_structure, gait_selection_map):
        for gait_name, gait in gait_directory_structure.iteritems():
            try:
                selection_map = gait_selection_map[gait_name]
            except KeyError:
                selection_map = None
            gait_preview = self._widget.findChild(QCheckBox, "ShowPreview").isChecked()

            gait_group_box = self.create_gait(gait_name, gait, selection_map, gait_preview)
            self._widget.Gaits.layout().addWidget(gait_group_box)

    def create_gait(self, name, gait, selections, gait_preview=False):
        gait_group_box = QGroupBox()
        gait_group_box.setObjectName("Gait")
        gait_group_box.setLayout(QHBoxLayout())

        gait_group_box.setTitle(name)
        image = QLabel()
        image.setStyleSheet(
            "background: url(" + gait["image"] + ") no-repeat center center 100px 100px;")
        image.setFixedSize(64, 80)
        gait_group_box.layout().addWidget(image)

        if gait_preview:
            from_subgait =  ["start",      "right_open", "left_swing",  "right_swing", "left_swing", "right_close", "right_swing", "left_close"]
            to_subgait  =   ["right_open", "left_swing", "right_swing", "left_swing" , "right_close", "end",        "left_close", "end"]
            graph = zip(from_subgait, to_subgait)

            gait_preview = self.create_gait_preview(graph)

            gait_group_box.layout().addWidget(gait_preview)

        for subgait_name, subgait in gait["subgaits"].iteritems():
            subgait_group_box = self.create_subgait(subgait_name, subgait, selections)
            gait_group_box.layout().addWidget(subgait_group_box)

        return gait_group_box

    def create_subgait(self, name, subgait, version_selection):
        subgait_group_box = QGroupBox()
        subgait_group_box.setLayout(QGridLayout())
        subgait_group_box.setObjectName("Subgait")
        subgait_group_box.setTitle(name)
        try:
            version_name = version_selection[name]
        except TypeError:
            version_name = None
        except KeyError:
            version_name = None
        dropdown = self.create_dropdown(subgait, version_name)

        subgait_group_box.layout().addWidget(dropdown, 0, 0)
        return subgait_group_box

    def create_dropdown(self, options, selection):
        try:
            index = options.index(selection)
        except ValueError:
            rospy.loginfo("Selection %s not found in options %s.", str(selection), str(options))
            index = -1
        dropdown = QComboBox()
        for option in options:
            dropdown.addItem(option)
        dropdown.setCurrentIndex(index)
        return dropdown

    def set_gait_selection_map(self, notify = False):
        gait_selection_map = {}
        gaits = self._widget.Gaits.findChildren(QGroupBox, "Gait")
        for gait in gaits:
            gait_name = str(gait.title())
            gait_selection_map[gait_name] = {}

            subgaits = gait.findChildren(QGroupBox, "Subgait")
            for subgait in subgaits:
                subgait_name = str(subgait.title())
                version = str(subgait.findChild(QComboBox).currentText())
                gait_selection_map[gait_name][subgait_name] = version

        res = self.set_version_map(str(gait_selection_map))
        if res.success:
            if notify:
                self.log("Selection applied.", Color.Debug)
            rospy.loginfo(res.message)
        else:
            self.log(res.message, Color.Error)
            self.log("Resetting to last valid selection", Color.Warning)
            rospy.logwarn(res.message)

    def update_defaults(self, notify=False):
        res = self.update_default_versions()
        if res.success:
            if notify:
                self.log("Default selection updated.", Color.Debug)
            rospy.loginfo(res.message)
        else:
            self.log(res.message, Color.Error)
            self.log("Resetting to last valid selection", Color.Warning)
            rospy.logwarn(res.message)

    def create_gait_preview(self, graph):
        G = nx.DiGraph()
        G.add_edges_from(graph)
        graph_items = nx.kamada_kawai_layout(G)

        background_color = pyqtgraph.mkColor("#F2F1F0")
        pyqtgraph.setConfigOption('background', background_color)
        pyqtgraph.setConfigOptions(antialias=True)
        plot_widget = pyqtgraph.PlotWidget()
        plot_widget.setFixedSize(300, 200)
        plot_widget.setDisabled(True)
        plot_widget.plotItem.hideAxis('bottom')
        plot_widget.plotItem.hideAxis('left')

        # Draw lines
        for node in graph:
            from_pos = graph_items[node[0]]
            to_pos = graph_items[node[1]]
            x1 = from_pos[0]
            y1 = from_pos[1]
            x2 = to_pos[0]
            y2 = to_pos[1]
            plot_widget.plot([x1, x2], [y1, y2], pen=pyqtgraph.mkPen(color='k', width=3))

        # Draw arrowheads, in a different loop so lines don't get drawn over them.
        for node in graph:
            from_pos = graph_items[node[0]]
            to_pos = graph_items[node[1]]
            x1 = from_pos[0]
            y1 = from_pos[1]
            x2 = to_pos[0]
            y2 = to_pos[1]
            angle = 180 - math.degrees(math.atan2((y2-y1),(x2-x1)))
            plot_widget.addItem(pyqtgraph.ArrowItem(pos=(x2, y2), angle=angle, brush=pyqtgraph.mkBrush(color='k'), pen=pyqtgraph.mkPen(color='k')))

        # Draw text
        for key, value in graph_items.iteritems():
            graph_item = pyqtgraph.TextItem(text=key, anchor=(1, 1), color='k')
            graph_item.setPos(value[0], value[1])
            plot_widget.addItem(graph_item)

        return plot_widget