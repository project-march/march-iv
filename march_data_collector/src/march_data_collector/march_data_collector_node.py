#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature, Imu
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState
from geometry_msgs.msg import TransformStamped
from tf.transformations import *
from math import pi
import tf2_ros


def TemperatureCallback(data, joint):
    rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))


def TrajectoryStateCallback(data):
    rospy.logdebug('received trajectory state' + str(data.desired))


def ImcStateCallback(data):
    rospy.logdebug('received IMC message current is ' + str(data.current))


def IMUCallback(data, IMUbroadcaster):
    if data.header.frame_id == "imu_link":
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "imu_link"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        imu_rotation = quaternion_multiply([-data.orientation.x, -data.orientation.y, data.orientation.z,
                                            data.orientation.w], quaternion_from_euler(0, -0.5*pi, 0))
        transform.transform.rotation.x = imu_rotation[0]
        transform.transform.rotation.y = imu_rotation[1]
        transform.transform.rotation.z = imu_rotation[2]
        transform.transform.rotation.w = imu_rotation[3]

        IMUbroadcaster.sendTransform(transform)


def main():
    rospy.init_node('data_collector', anonymous=True)
    joint_names = rospy.get_param('/march/joint_names')

    IMUbroadcaster = tf2_ros.TransformBroadcaster()

    TemperatureSubscriber = [rospy.Subscriber('/march/temperature/'+joint, Temperature, TemperatureCallback,
                                              (joint)) for joint in joint_names]

    TrajectoryStateSubscriber = rospy.Subscriber('/march/controller/trajectory/state', JointTrajectoryControllerState,
                                                 TrajectoryStateCallback)

    IMCStateSubscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, ImcStateCallback)

    IMUSubscriber = rospy.Subscriber('/march/imu', Imu, IMUCallback, (IMUbroadcaster))

    rospy.spin()
