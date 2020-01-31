import datetime
import logging
import os
import sys

from control_msgs.msg import JointTrajectoryControllerState
import rospy
from sensor_msgs.msg import Imu, Temperature
from visualization_msgs.msg import Marker

from march_shared_resources.msg import GaitNameActionGoal, ImcErrorState


try:
    sys.path.append(os.environ['DFESP_HOME'] + '/lib')
    import pubsubApi
    import modelingApi
except (ImportError, KeyError) as e:
    rospy.logerr('Error while loading libraries for ESP.\nProbably ESP is not installed on this machine.')
    sys.exit()


class ESPAdapter:

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
        self.subscribers = {}

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

        self.source_windows_esp = set(convert_stringv(stringv, True))

        for joint in joint_names:
            self.configure_source('sourceTemperature_' + joint, '/march/temperature/', Temperature,
                                  self.temperature_callback)

        self.configure_source('sourceJoint', '/march/controller/trajectory/state', JointTrajectoryControllerState,
                              self.trajectory_state_callback)
        self.configure_source('sourceIMU', '/march/imu', Imu, self.imu_callback)

        self.configure_source('sourceIMC', '/march/imc_states', ImcErrorState, self.imc_state_callback)
        self.configure_source('sourceGait', '/march/gait/schedule/goal', GaitNameActionGoal, self.gait_callback)
        self.configure_source('sourceCom', '/march/com_marker', Marker, self.com_callback)

    def pub_err_cb_func(self, failure, code, _):
        if failure == pubsubApi.pubsubFail_APIFAIL and code == pubsubApi.pubsubCode_CLIENTEVENTSQUEUED:
            return

        fail_msg = pubsubApi.DecodeFailure(failure)
        code_msg = pubsubApi.DecodeFailureCode(code)
        rospy.logerr('Client services error: ' + fail_msg + code_msg)

    def configure_source(self, source, topic, msg_type, callback):
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

        pub = pubsubApi.PublisherStart(window_url, pubsubApi.ERRCBFUNC(self.pub_err_cb_func), None)
        if pub is None:
            rospy.logwarn('Could not create ESP publisher client for source' + source)
            return

        ret = pubsubApi.Connect(pub)
        if ret != 1:
            rospy.logwarn('Could not connect ESP publisher client for source ' + source)
            return

        self.esp_publishers[source] = (pub, schemaptr)
        self.subscribers[source] = rospy.Subscriber(topic, msg_type, callback, source)

    def send_to_esp(self, csv, source):
        csv = 'i, n, 1,' + csv
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
        return ret == 1

    def temperature_callback(self, data, source):
        timestr = get_time_str(data.header.stamp)
        csv = timestr + ',' + str(data.temperature)
        self.send_to_esp(csv, source)

    def trajectory_state_callback(self, data, source):
        actual_positions_str = '[' + ';'.join([str(value) for value in data.actual.positions]) + ']'
        actual_velocity_str = '[' + ';'.join([str(value) for value in data.actual.velocities]) + ']'
        desired_positions_str = '[' + ';'.join([str(value) for value in data.desired.positions]) + ']'
        desired_velocity_str = '[' + ';'.join([str(value) for value in data.desired.velocities]) + ']'
        timestr = get_time_str(data.header.stamp)

        csv = ','.join([timestr, actual_positions_str, actual_velocity_str, desired_positions_str,
                        desired_velocity_str])
        self.send_to_esp(csv, source)

    def imu_callback(self, data, source):
        orientation_str = quaternion_to_str(data.orientation)
        angular_velocity_str = vector_to_str(data.angular_velocity)
        linear_acceleration_str = vector_to_str(data.linear_acceleration)
        timestr = get_time_str(data.header.stamp)

        csv = ','.join([timestr, orientation_str, angular_velocity_str, linear_acceleration_str])
        self.send_to_esp(csv, source)

    def imc_state_callback(self, data, source):
        motor_current_str = '[' + ';'.join([str(value) for value in data.motor_current]) + ']'
        motor_voltage_str = '[' + ';'.join([str(value) for value in data.motor_voltage]) + ']'
        timestr = get_time_str(data.header.stamp)

        csv = ','.join([timestr, motor_voltage_str, motor_current_str])
        self.send_to_esp(csv, source)

    def gait_callback(self, data, source):
        csv = ','.join([get_time_str(data.header.stamp), data.goal.name, data.goal.current_subgait.name,
                       data.goal.current_subgait.version])
        self.send_to_esp(csv, source)

    def com_callback(self, data, source):
        com_position_str = quaternion_to_str(data.pose.position)
        csv = ','.join([get_time_str(data.header.stamp), com_position_str])
        self.send_to_esp(csv, source)


def get_time_str(timestamp):
    time = timestamp.secs + timestamp.nsecs * 10**(-9)
    return datetime.datetime.fromtimestamp(time).strftime('%Y-%m-%d %H:%M:%S.%f')


def quaternion_to_str(quaternion):
    ls = [str(quaternion.x), str(quaternion.y), str(quaternion.z), str(quaternion.w)]
    return '[' + ';'.join(ls) + ']'


def vector_to_str(vector):
    ls = [str(vector.x), str(vector.y), str(vector.z)]
    return '[' + ';'.join(ls) + ']'


def convert_stringv(stringv, free):
    ls = [modelingApi.StringVGet(stringv, i) for i in range(0, modelingApi.StringVSize(stringv))]
    if free:
        modelingApi.StringVFree(stringv)
    return ls


def main():
    rospy.init_node('esp_adapter', anonymous=True)
    ESPAdapter()
    rospy.spin()
