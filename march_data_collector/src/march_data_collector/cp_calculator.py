from math import sqrt

import rospy
from visualization_msgs.msg import Marker


class CPCalculator(object):

    def __init__(self, com_mark, tf_buffer):
        self.tf_buffer = tf_buffer
        self.prev_x = com_mark.pose.position.x
        self.prev_y = com_mark.pose.position.y
        self.prev_t = com_mark.header.stamp

        self.markers = [Marker(), Marker()]
        for marker in self.markers:
            marker.header.frame_id = 'world'
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

        g = 9.81  # gravity constant
        z_zero = 0.715500019353  # height of CoM
        self.MULTIPLICATION_CONSTANT = sqrt(z_zero / g)

    def calculate_cp(self, com_mark):
        current_time = com_mark.header.stamp
        time_difference = (current_time - self.prev_t).to_sec()
        if current_time is not self.prev_t:
            x_dot = (com_mark.pose.position.x - self.prev_x) / time_difference
            y_dot = (com_mark.pose.position.y - self.prev_y) / time_difference
            # 0 is left 1 is right convention
            feet = ['ankle_plate_left', 'ankle_plate_right']
            for i in range(0, 2):
                marker = self.markers[i]
                trans = self.tf_buffer.lookup_transform('world', feet[i], rospy.Time())
                x_cap = trans.transform.translation.x + x_dot * self.MULTIPLICATION_CONSTANT
                y_cap = trans.transform.translation.y + y_dot * self.MULTIPLICATION_CONSTANT

                # send CP position to RViZ
                marker.header.stamp = rospy.get_rostime()
                marker.pose.position.x = x_cap
                marker.pose.position.y = y_cap
                marker.pose.position.z = 0
                rospy.logdebug('capture point is at ' + str(marker.pose.position))

            self.prev_x = com_mark.pose.position.x
            self.prev_y = com_mark.pose.position.y
            self.prev_t = current_time

        return self.markers
