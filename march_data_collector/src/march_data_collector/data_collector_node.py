import rospy
from sensor_msgs.msg import Temperature, Imu
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState
from geometry_msgs.msg import TransformStamped
from tf.transformations import *
from math import pi
from com_calculator import CoMCalculator
from visualization_msgs.msg import Marker
from urdf_parser_py.urdf import URDF
import tf2_ros


class DataCollectorNode(object):

    def __init__(self, com_calculator):
        self.com_calculator = com_calculator
        joint_names = rospy.get_param('/march/joint_names')
        self.imu_broadcaster = tf2_ros.TransformBroadcaster()

        self.marker_publisher = rospy.Publisher('/march/com_marker', Marker, queue_size=1)

        temperature_subscriber = [rospy.Subscriber('/march/temperature/' + joint, Temperature,
                                                   self.temperature_callback, joint) for joint in joint_names]

        trajectory_state_subscriber = rospy.Subscriber('/march/controller/trajectory/state',
                                                       JointTrajectoryControllerState, self.trajectory_state_callback)

        imc_state_subscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, self.imc_state_callback)

        imu_subscriber = rospy.Subscriber('/march/imu', Imu, self.imu_callback)

    def temperature_callback(self, data, joint):
        rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))

    def trajectory_state_callback(self, data):
        rospy.logdebug('received trajectory state' + str(data.desired))
        self.marker_publisher.publish(self.com_calculator.calculate_com())

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
                                                data.orientation.w], quaternion_from_euler(0, -0.5*pi, 0))
            transform.transform.rotation.x = imu_rotation[0]
            transform.transform.rotation.y = imu_rotation[1]
            transform.transform.rotation.z = imu_rotation[2]
            transform.transform.rotation.w = imu_rotation[3]

            self.imu_broadcaster.sendTransform(transform)


def main():
    rospy.init_node('data_collector', anonymous=True)

    robot = URDF.from_parameter_server()
    tf_buffer = tf2_ros.Buffer()
    center_of_mass_calculator = CoMCalculator(robot, tf_buffer)

    march_data_collector = DataCollectorNode(center_of_mass_calculator)
    rospy.spin()
