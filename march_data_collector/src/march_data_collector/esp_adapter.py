# This file contains ESP adapter class which sends data comming on ROS topic to an already running ESP enginge.
# It is assumed that the pubsub is opened on port 9901. The project is March_test and the continious query is March_cq.
# The connection is establish trough the libraries modelingApi.py and pubsubApi.py, which come with the installation and
# are a wrapper for the C Api. Detailed documentation about ESP can be found at
# https://documentation.sas.com/?cdcId=espcdc&cdcVersion=6.2&docsetId=espov&docsetTarget=home.htm&locale=nl

import datetime
import logging
import os
import sys

import rospy
from sensor_msgs.msg import Imu, Temperature
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from march_shared_resources.msg import GaitActionGoal, GaitActionResult, ImcErrorState, JointValues, PressureSole
from march_shared_resources.srv import CurrentState

try:
    sys.path.append(os.environ['DFESP_HOME'] + '/lib')
    import pubsubApi
    import modelingApi
except (ImportError, KeyError) as e:
    rospy.logerr('Error while loading libraries for ESP.\nProbably ESP is not installed on this machine.')
    sys.exit()


class ESPAdapter:
    """Class streams ros messages into an ESP engine."""

    def __init__(self):

        try:
            joint_names = rospy.get_param('/march/joint_names')
        except KeyError:
            joint_names = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'left_ankle', 'right_hip_aa',
                           'right_hip_fe', 'right_knee', 'right_ankle']
            rospy.loginfo('Cannot get joint_names from parameter server assuming usual 8: \n' + str(joint_names))

        ret = pubsubApi.Init(modelingApi.ll_Off, None)
        if ret == 0:
            rospy.logerr('Could not initialize pubsub library. \n killing ESP adapater')
            sys.exit()

        logger = logging.getLogger()
        logger.addHandler(modelingApi.getLoggingHandler())

        self.esp_publishers = {}
        self.ros_subscribers = {}

        basic_url = 'dfESP://localhost:9901'
        project = basic_url + '/March_test'
        self.contquery = project + '/March_cq'
        stringv = pubsubApi.QueryMeta(project + '?get=windows_sourceonly')
        if stringv is None:
            projects_ptr = pubsubApi.QueryMeta(basic_url + '?get=projects')
            if projects_ptr is None:
                rospy.logerr('Cannot connect to ESP server, is it running?\n killing ESP adapter')
            else:
                queries_ptr = pubsubApi.QueryMeta(project + '?get=projects')
                if queries_ptr is None:
                    rospy.logerr('Cannot connect to the desired project on the ESP server.\n killing ESP adapter')
                    rospy.loginfo('Possible projects are:\n' + str(convert_stringv(projects_ptr, True)))
                else:
                    rospy.logerr('Cannot connect to the desired continious query on the ESP server.\n '
                                 'killing ESP adapter')
                    rospy.loginfo('Possible continious queries are:\n' + str(convert_stringv(queries_ptr, True)))
            sys.exit()

        try:
            rospy.wait_for_service('march/state_machine/current_states', timeout=5.0)
            self.get_gait = rospy.ServiceProxy('march/state_machine/current_states', CurrentState, persistent=True)
        except rospy.exceptions.ROSException:
            rospy.loginfo('Service get current state not available using mock instead.')

            def mock_get_gait():
                """Mocks the get_gait ROS service when not available.

                :return: mock CurrentState msg
                """
                msg = CurrentState()
                msg.current_state = 'UNKNOWN'
                msg.state_type = 'idle'
                return msg
            self.get_gait = mock_get_gait

        self.source_windows_esp = set(convert_stringv(stringv, True))

        for joint in joint_names:
            self.configure_source('source_temperature_' + joint, '/march/temperature/' + joint, Temperature,
                                  self.temperature_callback)

        self.configure_source('source_imu', '/march/imu', Imu, self.imu_callback)

        self.configure_source('source_ps', '/march/pressure_soles', PressureSole, self.pressure_sole_callback)
        self.configure_source('source_imc', '/march/imc_states', ImcErrorState, self.imc_state_callback)
        self.configure_source('source_gait', '/march/gait/schedule/goal', GaitActionGoal, self.gait_callback)
        self.configure_source('source_gait', 'march/gait/perform/result', GaitActionResult, self.gait_finished_callback)
        self.configure_source('source_com', '/march/com_marker', Marker, self.com_callback)
        self.configure_source('source_joint', '/march/joint_values', JointValues, self.joint_values_callback)

        msg = GaitActionResult()
        msg.header.stamp = rospy.Time.now()
        self.gait_finished_callback(msg, 'source_gait')

    def configure_source(self, source, topic, msg_type, callback):
        """Configures a connection between a ROS topic and a source window in an event stream processing engine.

        :param source: the name of the source window in the ESP engine
        :param topic: the topic on which the ROS messages are posted
        :param msg_type: the type of the ROS messages
        :param callback: the callback function that can receive the ROS message and create a CSV string for the
        source window
        """
        if source not in self.source_windows_esp:
            rospy.logwarn('There is no ESP source window for the following source: ' + source)
            return

        window_url = self.contquery + '/' + source
        stringv = pubsubApi.QueryMeta(window_url + '?get=schema')

        if stringv is None:
            rospy.logwarn('Could not get ESP source window schema for window ' + source)
            modelingApi.StringVFree(stringv)
            return

        schema = modelingApi.StringVGet(stringv, 0)

        if schema is None:
            rospy.logwarn('Could not get ESP schema from query response for source ' + source)
            return

        schemaptr = modelingApi.SchemaCreate(source, schema)
        if schemaptr is None:
            rospy.logwarn('Could not build ESP source window schema for source ' + source)
            return

        pub = pubsubApi.PublisherStart(window_url, pubsubApi.ERRCBFUNC(pub_err_cb_func), None)
        if pub is None:
            rospy.logwarn('Could not create ESP publisher client for source' + source)
            return

        ret = pubsubApi.Connect(pub)
        if ret != 1:
            rospy.logwarn('Could not connect ESP publisher client for source ' + source)
            return

        self.esp_publishers[source] = (pub, schemaptr)
        self.ros_subscribers[source] = rospy.Subscriber(topic, msg_type, callback, source)
        rospy.logdebug('configured ESP sourcewindow for ' + source)

    def send_to_esp(self, csv, source):
        """Sends a csv string to the configured source window. Also adds standard stuff to the start of the csv string.

        :param csv: the csv string containing all data to send to the message
        :param source: the name of the source window in the ESP engine
        """
        csv = 'i, n, 1, 1,' + csv
        try:
            pub, schemaptr = self.esp_publishers[source]
        except KeyError:
            rospy.loginfo_throttle(3, 'Receiving data for ' + source + ', but cannot send to ESP, '
                                                                       'because the source window is not configured.')
            return False
        event = modelingApi.EventCreate2(schemaptr, csv, '%Y-%m-%d %H:%M:%S')
        event_vector = modelingApi.EventVCreate()
        modelingApi.EventVPushback(event_vector, event)
        event_block = modelingApi.EventBlockNew1(event_vector, modelingApi.ebt_NORMAL)
        ret = pubsubApi.PublisherInject(pub, event_block)
        modelingApi.EventBlockDestroy(event_block)
        if ret != 1:
            rospy.logwarn('Unsuccessful inject into ESP server for source window {source} and event {event}'.format(
                source=source, event=csv))

    def gait_finished_callback(self, data, source):
        """Callback for stopped gait. If the current state is an idle state send this to ESP a gait.

        :param data: ROS message
        :param source: the name of the source window in the ESP engine
        """
        rospy.sleep(0.03)
        state = self.get_gait()
        if 'idle' in state.state_type:
            csv = ','.join([get_time_str(data.header.stamp), 'idle', state.current_state.lower(), ' '])
            self.send_to_esp(csv, source)

    def temperature_callback(self, data, source):
        """Callback for temperature data. Converts ROS message to csv string to send to the source window.

        :param data: ROS temperature message
        :param source: the name of the source window in the ESP engine
        """
        time_str = get_time_str(data.header.stamp)
        csv = time_str + ',' + str(data.temperature)
        self.send_to_esp(csv, source)

    def joint_values_callback(self, data, source):
        """Callback for trajectory_state data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resources.msgs.JointValues
        :param source: the name of the source window in the ESP engine
        """
        actual_positions_str = list_to_str(data.controller_output.actual.positions)
        actual_velocity_str = list_to_str(data.velocities)
        acutal_acceleration_str = list_to_str(data.accelerations)
        acutal_jerk_str = list_to_str(data.jerks)
        desired_positions_str = list_to_str(data.controller_output.desired.positions)
        desired_velocity_str = list_to_str(data.controller_output.desired.velocities)
        position_error_str = list_to_str(data.controller_output.error.positions)
        time_str = get_time_str(data.controller_output.header.stamp)

        csv = ','.join([time_str, actual_positions_str, actual_velocity_str, acutal_acceleration_str, acutal_jerk_str,
                        desired_positions_str, desired_velocity_str, position_error_str])
        self.send_to_esp(csv, source)

    def imu_callback(self, data, source):
        """Callback for imu data. Converts ROS message to csv string to send to the source window.

        :param data: ROS sensor_msgs.Imu message
        :param source: the name of the source window in the ESP engine
        """
        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        orientation_str = list_to_str(euler_from_quaternion(orientation))

        angular_velocity_str = vector_to_str(data.angular_velocity)
        linear_acceleration_str = vector_to_str(data.linear_acceleration)
        time_str = get_time_str(data.header.stamp)

        csv = ','.join([time_str, orientation_str, angular_velocity_str, linear_acceleration_str])
        rospy.loginfo(csv)
        self.send_to_esp(csv, source)

    def imc_state_callback(self, data, source):
        """Callback for IMotionCube data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resources.msgs.ImcErrorState message
        :param source: the name of the source window in the ESP engine
        """
        motor_current_str = ','.join([str(value) for value in data.motor_current])
        motor_voltage_str = ','.join([str(value) for value in data.motor_voltage])
        time_str = get_time_str(data.header.stamp)

        csv = ','.join([time_str, motor_voltage_str, motor_current_str])
        self.send_to_esp(csv, source)

    def gait_callback(self, data, source):
        """Callback for gait data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resoruces.GaitActionGoal message
        :param source: the name of the source window in the ESP engine
        """
        csv = ','.join([get_time_str(data.header.stamp), data.goal.name, data.goal.current_subgait.name,
                        data.goal.current_subgait.version])
        self.send_to_esp(csv, source)

    def com_callback(self, data, source):
        """Callback for center of mass data. Converts ROS message to csv string to send to the source window.

        :param data: ROS visualization_msgs.Marker message
        :param source: the name of the source window in the ESP engine
        """
        csv = ','.join([get_time_str(data.header.stamp), str(data.pose.position.x), str(data.pose.position.y),
                        str(data.pose.position.z)])
        self.send_to_esp(csv, source)

    def pressure_sole_callback(self, data, source):
        """Callback for pressure sole data. Converts ROS message to csv string to send to the source window.

        :param data: ROS march_shared_resources.PressureSole message
        :param source: the name of the source window in the ESP engine
        """
        pressure_left = list_to_str(data.pressure_left)
        pressure_right = list_to_str(data.pressure_left)
        cop_left = list_to_str(data.cop_left)
        cop_right = list_to_str(data.cop_right)
        csv = ','.join([get_time_str(data.pressure_soles_time), str(data.total_force_left), str(data.total_force_right),
                        pressure_left, pressure_right, cop_left, cop_right])
        self.send_to_esp(csv, source)


def get_time_str(timestamp):
    """Creates str to use in csv string for source window based on timestamp.

    :param data: ROS timestamp message std_msgs/stamp
    """
    time = timestamp.secs + timestamp.nsecs * 10 ** (-9)
    return datetime.datetime.fromtimestamp(time).strftime('%Y-%m-%d %H:%M:%S.%f')


def list_to_array_str(ls):
    """Converts a list to an array formatted string for source window.

    :type ls: list
    """
    return '[' + ';'.join(str(value) for value in ls) + ']'


def list_to_str(ls):
    """Converts a list to a csv string for source window.

    :type ls: list
    """
    return ','.join(str(value) for value in ls)


def vector_to_str(vector):
    """Converts geometry_msgs/Vector3 to string to use in csv string for source window.

    :type vector: geometry_msgs/Vector3
    """
    return '{x},{y},{z}'.format(x=vector.x, y=vector.y, z=vector.z)


def convert_stringv(stringv, free):
    """Converts a stringV object (integer pointer, which points to a string vector via de API) to a list.

    :param data: stringV object from the modelingApi to convert
    :param free: whether to free the object afterwards
    """
    ls = [modelingApi.StringVGet(stringv, i) for i in range(0, modelingApi.StringVSize(stringv))]
    if free:
        modelingApi.StringVFree(stringv)
    return ls


def pub_err_cb_func(self, failure, code, _):
    if failure == pubsubApi.pubsubFail_APIFAIL and code == pubsubApi.pubsubCode_CLIENTEVENTSQUEUED:
        return

    fail_msg = pubsubApi.DecodeFailure(failure)
    code_msg = pubsubApi.DecodeFailureCode(code)
    rospy.logerr('Client services error: ' + fail_msg + code_msg)


def main():
    rospy.init_node('esp_adapter', anonymous=True)
    ESPAdapter()
    rospy.spin()
