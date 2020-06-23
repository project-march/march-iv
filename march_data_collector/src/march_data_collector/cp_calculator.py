from math import sqrt

from numpy import float64
from numpy_ringbuffer import RingBuffer
import rospy
from scipy.signal import savgol_filter
import tf2_ros
from visualization_msgs.msg import Marker


class CPCalculator(object):

    def __init__(self, tf_buffer, foot_link):
        self.tf_buffer = tf_buffer
        self.foot_link = foot_link
        self.publisher = rospy.Publisher('/march/cp_marker_' + foot_link, Marker, queue_size=1)
        self.publisher2 = rospy.Publisher('/march/cp_marker2_' + foot_link, Marker, queue_size=1)

        self.prev_x = 0
        self.prev_y = 0
        self.prev_t = rospy.Time.now()

        self.times = RingBuffer(capacity=10, dtype=float64)
        self.com_x = RingBuffer(capacity=10, dtype=float64)
        self.com_y = RingBuffer(capacity=10, dtype=float64)

        self.marker = Marker()

        self.marker.header.frame_id = 'world'
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        self.g = 9.81  # gravity constant

    def calculate_cp(self, com_mark):
        self.com_x.append(com_mark.pose.position.x)
        self.com_y.append(com_mark.pose.position.y)

        current_time = com_mark.header.stamp
        time_difference = (current_time - self.prev_t).to_sec()



        # rospy.loginfo(deriv_x[-1])
        # rospy.loginfo(deriv_y[-1])


        if current_time is not self.prev_t:
            x_dot = savgol_filter(self.com_x, window_length=3, polyorder=2, deriv=1, delta=time_difference, mode='nearest')[-1]
            y_dot = savgol_filter(self.com_y, window_length=3, polyorder=2, deriv=1, delta=time_difference, mode='nearest')[-1]
            x_dot2 = savgol_filter(self.com_x, window_length=9, polyorder=2, deriv=1, delta=time_difference, mode='interp')[-1]
            y_dot2 = savgol_filter(self.com_y, window_length=9, polyorder=2, deriv=1, delta=time_difference, mode='interp')[-1]

            # x_dot = (com_mark.pose.position.x - self.prev_x) / time_difference
            # y_dot = (com_mark.pose.position.y - self.prev_y) / time_difference

            try:
                trans = self.tf_buffer.lookup_transform('world', self.foot_link, rospy.Time())
                try:
                    multiplier = sqrt(com_mark.pose.position.z / self.g)
                except ValueError:
                    rospy.logdebug_throttle(1, 'Cannot calculate capture point, because center of mass height is '
                                               'smaller than 0')
                    return self.marker

                x_cp = trans.transform.translation.x + x_dot * multiplier
                y_cp = trans.transform.translation.y + y_dot * multiplier

                x_cp2 = trans.transform.translation.x + x_dot2 * multiplier
                y_cp2 = trans.transform.translation.y + y_dot2 * multiplier

                self.update_marker2(x_cp2, y_cp2)

                self.update_marker(x_cp, y_cp)

                # self.prev_x = com_mark.pose.position.x
                # self.prev_y = com_mark.pose.position.y
                # self.prev_t = current_time


            except tf2_ros.TransformException as e:
                rospy.logdebug('Error in trying to lookup transform for capture point: {error}'.format(error=e))

        return self.marker

    def update_marker(self, x_cp, y_cp):
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.pose.position.x = x_cp
        self.marker.pose.position.y = y_cp
        self.marker.pose.position.z = 0

        rospy.logdebug('capture point is at ' + str(self.marker.pose.position))

        self.publisher.publish(self.marker)

    def update_marker2(self, x_cp, y_cp):
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.pose.position.x = x_cp
        self.marker.pose.position.y = y_cp
        self.marker.pose.position.z = 0

        rospy.logdebug('capture point is at ' + str(self.marker.pose.position))

        self.publisher2.publish(self.marker)
