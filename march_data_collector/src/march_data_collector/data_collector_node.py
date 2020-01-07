from math import pi

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import Imu, Temperature
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

from march_shared_resources.msg import ImcErrorState

from .com_calculator import CoMCalculator
from .cp_calculator import CPCalculator

# standard imports
from ctypes import *
import os
import sys
import platform
import logging
dfesphome = os.environ["DFESP_HOME"]
sys.path.append(dfesphome + '/lib')
import pubsubApi
import modelingApi
import pubsubInternal

import lxml.etree as etree

import socket




class DataCollectorNode(object):

    def __init__(self, com_calculator, cp_calculators):
        self._com_calculator = com_calculator
        self._cp_calculators = cp_calculators

        joint_names = rospy.get_param('/march/joint_names')

        self._imu_broadcaster = tf2_ros.TransformBroadcaster()
        self._com_marker_publisher = rospy.Publisher('/march/com_marker', Marker, queue_size=1)

        self._temperature_subscriber = [rospy.Subscriber('/march/temperature/' + joint,
                                                         Temperature,
                                                         self.temperature_callback, joint) for joint in joint_names]

        self._trajectory_state_subscriber = rospy.Subscriber('/march/controller/trajectory/state',
                                                             JointTrajectoryControllerState,
                                                             self.trajectory_state_callback)

        self._imc_state_subscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, self.imc_state_callback)

        self._imu_subscriber = rospy.Subscriber('/march/imu', Imu, self.imu_callback)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = "localhost"
        self.port = 32000
        # sock.sendto(msg.encode('utf-8'), (host, port))










    def send_udp(self, msg):
        self.sock.sendto(msg.encode('utf-8'), (self.host, self.port))


    def temperature_callback(self, data, joint):
        rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))

    def trajectory_state_callback(self, data):
        rospy.logdebug('received trajectory state' + str(data.desired))
        joint_angles = data.actual.positions
        angle_str = " ".join([str(angle) for angle in joint_angles])
        msg = " ".join(["joinst_angles", str(data.header.stamp.secs), angle_str])

        self.send_udp(msg)


        com = self._com_calculator.calculate_com()
        self._com_marker_publisher.publish(com)
        for cp_calculator in self._cp_calculators:
            cp_calculator.calculate_cp(com)

    def imc_state_callback(self, data):
        rospy.logdebug('received IMC message current is ' + str(data.current))

    def imu_callback(self, data):
        if data.header.frame_id == 'imu_link':
            transform = TransformStamped()

            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'imu_link'
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0

            imu_rotation = quaternion_multiply([-data.orientation.x, -data.orientation.y, data.orientation.z,
                                                data.orientation.w], quaternion_from_euler(0, -0.5 * pi, 0))
            transform.transform.rotation.x = imu_rotation[0]
            transform.transform.rotation.y = imu_rotation[1]
            transform.transform.rotation.z = imu_rotation[2]
            transform.transform.rotation.w = imu_rotation[3]

            self._imu_broadcaster.sendTransform(transform)



def main():
    rospy.init_node('data_collector', anonymous=True)

    robot = URDF.from_parameter_server()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    center_of_mass_calculator = CoMCalculator(robot, tf_buffer)
    feet = ['ankle_plate_left', 'ankle_plate_right']
    cp_calculators = [CPCalculator(tf_buffer, foot) for foot in feet]

    DataCollectorNode(center_of_mass_calculator, cp_calculators)

    rospy.spin()
