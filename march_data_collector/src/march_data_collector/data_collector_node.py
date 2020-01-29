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
import datetime





class DataCollectorNode(object):

    # def __init__(self,  com_calculator, cp_calculators):
    def __init__(self):
        # joint_names = rospy.get_param('/march/joint_names')
        joint_names = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'left_ankle', 'right_hip_aa',
                       'right_hip_fe', 'right_knee', 'right_ankle']


        # below should match with the source windows in the xml file
        sourceWindows = ["sourceWindowJoint"] + ["sourceWindowTemperature_" + joint for joint in joint_names]
        print("source windows according to ros\n")
        print(sourceWindows)

        def pubErrCbFunc(failure, code, ctx):
            # don't output anything for client busy
            if failure == pubsubApi.pubsubFail_APIFAIL and code == pubsubApi.pubsubCode_CLIENTEVENTSQUEUED:
                return

            failMsg = pubsubApi.DecodeFailure(failure)
            codeMsg = pubsubApi.DecodeFailureCode(code)
            print('Client services error: ' + failMsg + codeMsg)

        ret = pubsubApi.Init(modelingApi.ll_Off, None)
        if ret == 0:
            rospy.logwarn(" Could not initialize pubsub library")
        logger = logging.getLogger()
        logger.addHandler(modelingApi.getLoggingHandler())

        pubErrFunc = pubsubApi.ERRCBFUNC(pubErrCbFunc)

        basic_url = 'dfESP://localhost:9901'
        project = basic_url + '/March_test'
        contquery = project + '/March_cq'

        stringv = pubsubApi.QueryMeta(project+"?get=windows_sourceonly")
        sourceWindow_esp = [modelingApi.StringVGet(stringv, i) for i in range(0, modelingApi.StringVSize(stringv))]
        print("source windows ESP\n")
        print(sourceWindow_esp)
        print(set(sourceWindows)-set(sourceWindow_esp))


        self.esp_publishers = {}
        for source in sourceWindows:
            window_url = contquery + "/" + source
            stringv = pubsubApi.QueryMeta(window_url+"?get=schema")
            if stringv == None:
                rospy.logwarn('Could not get ESP source window schema for window ' +source)
                continue
                # pubsubApi.Shutdown()

            schema = modelingApi.StringVGet(stringv, 0)

            if schema == None:
                rospy.logwarn('Could not get ESP schema from query response for source ' + source)
                continue
            schemaptr = modelingApi.SchemaCreate(source, schema)

            if schemaptr == None:
                rospy.logwarn('Could not build ESP source window schema for source ' + source)
                continue

            pub = pubsubApi.PublisherStart(window_url, pubErrFunc, None)
            if pub == None:
                rospy.logwarn('Could not create ESP publisher client for source' + source)
                continue

            ret = pubsubApi.Connect(pub)
            if ret != 1:
                logger.error('Could not connect ESP publisher client for source ' + source)
                pubsubApi.Stop(pub, 0)
                continue
                pubsubApi.Shutdown()

            self.esp_publishers[source] = (pub, schemaptr)

        print(self.esp_publishers)
        modelingApi.StringVFree(stringv)


        # self._imu_broadcaster = tf2_ros.TransformBroadcaster()
        # self._com_marker_publisher = rospy.Publisher('/march/com_marker', Marker, queue_size=1)
        #
        self._temperature_subscriber = [rospy.Subscriber('/march/temperature/' + joint,
                                                         Temperature,
                                                         self.temperature_callback, joint) for joint in joint_names]

        self._trajectory_state_subscriber = rospy.Subscriber('/march/controller/trajectory/state',
                                                             JointTrajectoryControllerState,
                                                             self.trajectory_state_callback)

        # self._imc_state_subscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, self.imc_state_callback)
        #
        self._imu_subscriber = rospy.Subscriber('/march/imu', Imu, self.imu_callback)



        # pubsubApi.Stop(pub, 1)
        # pubsubApi.Shutdown()


    def send_to_esp(self, csv, source):
        rospy.loginfo(csv)
        csv = 'i, n, 1,' + csv
        pub, schemaptr = self.esp_publishers[source]
        event = modelingApi.EventCreate2(schemaptr, csv, "%Y-%m-%d %H:%M:%S")
        eventVector = modelingApi.EventVCreate()
        modelingApi.EventVPushback(eventVector, event)
        eventBlock = modelingApi.EventBlockNew1(eventVector, modelingApi.ebt_NORMAL)

        ret = pubsubApi.PublisherInject(pub, eventBlock)
        print ("Return value inject = " +str(ret))
        modelingApi.EventBlockDestroy(eventBlock)


    def temperature_callback(self, data, joint):
        # rospy.logwarn("Temperature")
        time = data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)
        timestr = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S.%f")
        csv = timestr + ',' + str(data.temperature)
        self.send_to_esp(csv, "sourceWindowTemperature_" + joint)


    def trajectory_state_callback(self, data):
        joint_angles = data.actual.positions
        time = data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)
        timestr = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S.%f")
        csv = timestr + ',[' + ";".join([str(value) for value in joint_angles]) + "]"
        # self.send_to_esp(csv, "sourceWindowJoint")
        #
        # com = self._com_calculator.calculate_com()
        # self._com_marker_publisher.publish(com)
        # for cp_calculator in self._cp_calculators:
        #     cp_calculator.calculate_cp(com)

    # def imc_state_callback(self, data):
    #     rospy.logdebug('received IMC message current is ' + str(data.current))
    #
    def imu_callback(self, data):
        if data.header.frame_id == 'imu_link':
            rospy.loginfo(data)
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

    # robot = URDF.from_parameter_server()
    # tf_buffer = tf2_ros.Buffer()
    # tf2_ros.TransformListener(tf_buffer)
    # center_of_mass_calculator = CoMCalculator(robot, tf_buffer)
    # feet = ['ankle_plate_left', 'ankle_plate_right']
    # cp_calculators = [CPCalculator(tf_buffer, foot) for foot in feet]

    # DataCollectorNode(center_of_mass_calculator, cp_calculators)
    DataCollectorNode()
    rospy.spin()
