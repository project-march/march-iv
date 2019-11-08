#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState
from geometry_msgs.msg import TransformStamped
import tf2_ros


def TemperatureCallback(data, joint):
    rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))


def TrajectoryStateCallback(data):
    rospy.logdebug('received trajectory state' + str(data.desired))


def ImcStateCallback(data):
    rospy.logdebug('received IMC message current is ' + str(data.current))

def IMUCallback(data, IMUbroadcaster):
    #create tf frame
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "hip_base"
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = data.orientation.z
    transform.transform.rotation.y = data.orientation.y
    transform.transform.rotation.z = data.orientation.x
    transform.transform.rotation.w = data.orientation.w

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

    IMUSubscriber = rospy.Subscriber('/march/imu/00B447AD', Imu, IMUCallback, (IMUbroadcaster))

    rospy.spin()
