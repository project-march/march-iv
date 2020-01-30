import os
import sys
import logging
import rospy

from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import Imu, Temperature
from visualization_msgs.msg import Marker

from march_shared_resources.msg import ImcErrorState, GaitNameActionGoal


try:
    sys.path.append(os.environ["DFESP_HOME"] + '/lib')
    import pubsubApi
    import modelingApi
except ImportError as e:
    rospy.logerr("\n Probably ESP is not installed on this machine \n" + str(e))
    sys.exit()

import datetime

class ESPAdapter:

    def __init__(self):
        try:
            joint_names = rospy.get_param('/march/joint_names')
        except KeyError:
            joint_names = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'left_ankle', 'right_hip_aa',
                           'right_hip_fe', 'right_knee', 'right_ankle']
            rospy.loginfo("Cannot get joint_names from parameter server assuming usual 8: \n" + str(joint_names))

        ret = pubsubApi.Init(modelingApi.ll_Off, None)
        if ret == 0:
            rospy.logerr("Could not initialize pubsub library. \n killing ESP adapater")
            sys.exit()

        # below should match with the source windows in the march.xml file
        sourceWindows = {"sourceWindowJoint", "sourceWindowIMU", "sourceWindowIMC, sourceWindowGait, sourceWindowCom"}\
                        | set(["sourceWindowTemperature_" + joint for joint in joint_names])

        def pubErrCbFunc(failure, code, ctx):
            if failure == pubsubApi.pubsubFail_APIFAIL and code == pubsubApi.pubsubCode_CLIENTEVENTSQUEUED:
                return

            failMsg = pubsubApi.DecodeFailure(failure)
            codeMsg = pubsubApi.DecodeFailureCode(code)
            print('Client services error: ' + failMsg + codeMsg)

        pubErrFunc = pubsubApi.ERRCBFUNC(pubErrCbFunc)

        logger = logging.getLogger()
        logger.addHandler(modelingApi.getLoggingHandler())

        self.esp_publishers = {}
        self.subscribers = {}

        basic_url = 'dfESP://localhost:9901'
        project = basic_url + '/March_test'
        contquery = project + '/March_cq'

        stringv = pubsubApi.QueryMeta(project+"?get=windows_sourceonly")
        if stringv == None:
            #something failed do more debugging
            projects_ptr = pubsubApi.QueryMeta(basic_url + "?get=projects")
            if projects_ptr == None:
                rospy.logerr("Cannot connect to ESP server, is it running?\n killing ESP adapter")
            else:
                queries_ptr = pubsubApi.QueryMeta(project + "?get=projects")
                if queries_ptr == None:
                    rospy.logerr("Cannot connect to the desired project on the ESP server.\n killing ESP adapter")
                    rospy.loginfo("Possible projects are:\n" + str(convert_stringV(projects_ptr, True)))
                else:
                    rospy.logerr("Cannot connect to the desired continious query on the ESP server.\n killing ESP adapter")
                    rospy.loginfo("Possible continious queries are:\n" + str(convert_stringV(queries_ptr, True)))
            sys.exit()

        sourceWindow_esp = set([modelingApi.StringVGet(stringv, i) for i in range(0, modelingApi.StringVSize(stringv))])
        modelingApi.StringVFree(stringv)
        missing_windows = sourceWindows - sourceWindow_esp

        sources = sourceWindows & sourceWindow_esp
        if len(missing_windows) != 0:
            rospy.logwarn("Source windows missing in ESP model: " + str(missing_windows))
        rospy.logdebug("Configuring sourceWindows ESP for the following sources:\n " + str(sources))
        for source in sources:
            window_url = contquery + "/" + source
            stringv = pubsubApi.QueryMeta(window_url+"?get=schema")

            if stringv == None:
                rospy.logwarn('Could not get ESP source window schema for window ' +source)
                modelingApi.StringVFree(stringv)
                continue

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
                continue

            self.esp_publishers[source] = (pub, schemaptr)
            self.subscribers[source] = self.create_subscriber(source)


    def create_subscriber(self, source):
        if source == "sourceWindowJoint":
            return rospy.Subscriber('/march/controller/trajectory/state', JointTrajectoryControllerState,
                                    self.trajectory_state_callback, source)

        if source.startswith("sourceWindowTemperature_"):
            joint = source[len("sourceWindowTemperature_"):]
            return rospy.Subscriber('/march/temperature/' + joint, Temperature, self.temperature_callback, source)

        if source=="sourceWindowIMU":
            return rospy.Subscriber('/march/imu', Imu, self.imu_callback, source)

        if source=="sourceWindowIMC":
            return rospy.Subscriber('/march/imc_states', ImcErrorState, self.imc_state_callback, source)

        if source=="sourceWindowGait":
            return rospy.Subscriber('/march/gait/schedule/goal', GaitNameActionGoal, self.gait_callback)

        if source=="sourceWindowCom":
            return rospy.Subscriber('/march/com_marker', Marker, self.com_callback)


    def send_to_esp(self, csv, source):
        csv = 'i, n, 1,' + csv
        try:
            pub, schemaptr = self.esp_publishers[source]
        except KeyError:
            rospy.loginfo_throttle(3, "Receiving data for " + source + ", but cannot send to ESP, "
                                                                       "because the source window is not configured.")
        event = modelingApi.EventCreate2(schemaptr, csv, "%Y-%m-%d %H:%M:%S")
        eventVector = modelingApi.EventVCreate()
        modelingApi.EventVPushback(eventVector, event)
        eventBlock = modelingApi.EventBlockNew1(eventVector, modelingApi.ebt_NORMAL)
        ret = pubsubApi.PublisherInject(pub, eventBlock)
        modelingApi.EventBlockDestroy(eventBlock)
        return ret

    def temperature_callback(self, data, source):
        timestr = get_time_str(data.header.stamp)
        csv = timestr + ',' + str(data.temperature)
        self.send_to_esp(csv, source)


    def trajectory_state_callback(self, data, source):
        actual_positions_str = '[' + ";".join([str(value) for value in data.actual.positions]) + ']'
        actual_velocity_str = '[' + ";".join([str(value) for value in data.actual.velocities]) + ']'
        desired_positions_str = '[' + ";".join([str(value) for value in data.desired.positions]) + ']'
        desired_velocity_str = '[' + ";".join([str(value) for value in data.desired.velocities]) + ']'
        timestr = get_time_str(data.header.stamp)

        csv = ",".join([timestr, actual_positions_str, actual_velocity_str, desired_positions_str, desired_velocity_str])
        self.send_to_esp(csv, source)

    def imu_callback(self, data, source):
        orienatation_str = quaternion_to_str(data.orientation)
        angular_velocity_str = vector_to_str(data.angular_velocity)
        linear_acceleration_str = vector_to_str(data.linear_acceleration)
        timestr = get_time_str(data.header.stamp)

        csv = ",".join([timestr, orienatation_str, angular_velocity_str, linear_acceleration_str])
        self.send_to_esp(csv, source)

    def imc_state_callback(self, data, source):
        motor_current_str = '[' + ";".join([str(value) for value in data.motor_current]) + ']'
        motor_voltage_str = '[' + ";".join([str(value) for value in data.motor_voltage]) + ']'
        timestr = get_time_str(data.header.stamp)

        csv = ",".join([timestr, motor_voltage_str,motor_current_str ])
        self.send_to_esp(csv, source)

    def gait_callback(self, data, source):
        csv = ",".join([get_time_str(data.header.stamp), data.goal.name, data.goal.current_subgait.name,
                       data.goal.current_subgait.version])
        self.send_to_esp(csv, source)

    def com_callback(self, data, source):
        com_position_str = quaternion_to_str(data.pose.position)
        csv = ",".join([get_time_str(data.header.stamp), com_position_str])
        self.send_to_esp(csv, source)

def get_time_str(timestamp):
    time = timestamp.secs + timestamp.nsecs * 10**(-9)
    return datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S.%f")

def quaternion_to_str(quaternion):
    list = [str(quaternion.x), str(quaternion.y), str(quaternion.z), str(quaternion.w)]
    return "[" + ";".join(list) + "]"

def vector_to_str(quaternion):
    list = [str(quaternion.x), str(quaternion.y), str(quaternion.z)]
    return "[" + ";".join(list) + "]"

def convert_stringV(stringV, free):
    list = [modelingApi.StringVGet(stringV, i) for i in range(0, modelingApi.StringVSize(stringV))]
    if free:
        modelingApi.StringVFree(stringV)
    return list

def main():
    rospy.init_node('esp_adapter', anonymous=True)
    ESPAdapter()
    rospy.spin()