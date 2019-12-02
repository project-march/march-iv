from math import sqrt

import rospy
from visualization_msgs.msg import Marker


class CPCalculator(object):

    def __init__(self, com_mark):
        self.prev_x = com_mark.pose.position.x
        self.prev_y = com_mark.pose.position.y
        self.prev_t = com_mark.header.stamp

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

        g = 9.81  # gravity constant
        z_zero = 0.715500019353  # height of CoM
        self.MULTIPLICATION_CONSTANT = sqrt(z_zero / g)

    def calculate_cp(self, com_mark):
        x_dot = 0
        y_dot = 0

        current_time = com_mark.header.stamp
        time_difference = (current_time - self.prev_t).to_sec()
        if current_time is not self.prev_t:
            x_dot = (com_mark.pose.position.x - self.prev_x) / (time_difference)
            y_dot = (com_mark.pose.position.y - self.prev_y) / (time_difference)

        x_cap = com_mark.pose.position.x + x_dot * self.MULTIPLICATION_CONSTANT
        y_cap = com_mark.pose.position.y + y_dot * self.MULTIPLICATION_CONSTANT

        # send CP position to RViZ
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.pose.position.x = x_cap
        self.marker.pose.position.y = y_cap
        self.marker.pose.position.z = 0
        rospy.logdebug('capture point is at ' + str(self.marker.pose.position))

        self.prev_x = com_mark.pose.position.x
        self.prev_y = com_mark.pose.position.y
        self.prev_t = current_time

        return self.marker
