#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState
from CoMCalculator import CoMCalculator
from visualization_msgs.msg import Marker
from urdf_parser_py.urdf import URDF
import tf2_ros


def TemperatureCallback(data, joint):
    rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))


def TrajectoryStateCallback(data, args):
    rospy.logdebug('received trajectory state' + str(data.desired))
    args[0].publish(args[1].calculateCoM())



def ImcStateCallback(data):
    rospy.logdebug('received IMC message current is ' + str(data.current))


def main():
    rospy.init_node('data_collector', anonymous=True)
    joint_names = rospy.get_param('/march/joint_names')
    TemperatureSubscriber = [rospy.Subscriber('/march/temperature/'+joint, Temperature, TemperatureCallback,
                                              (joint)) for joint in joint_names]


    robot = URDF.from_parameter_server()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    CenterOfMassCalculator = CoMCalculator(robot, tfListener, tfBuffer)
    TrajectoryStateSubscriber = rospy.Subscriber('/march/controller/trajectory/state', JointTrajectoryControllerState,
                                                 TrajectoryStateCallback, (MarkerPublisher, CenterOfMassCalculator))

    IMCStateSubscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, ImcStateCallback)



