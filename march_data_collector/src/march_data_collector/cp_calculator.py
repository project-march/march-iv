# Copyright (c) Hamburg Bit-Bots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
# of the Software, and to permit persons to whom the Software is furnished to do
# so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import rospy
from math import sqrt
from visualization_msgs.msg import Marker


class CPCalculator(object):

    def __init__(self, com_mark):
        self.prev_x = com_mark.pose.position.x
        self.prev_y = com_mark.pose.position.y
        self.prev_t = com_mark.header.stamp.nsecs

        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        self.g = 9.81
        self.z_zero = 0.715500019353

    def calculate_cp(self, com_mark):
        x_dot = 0
        y_dot = 0

        current_time = com_mark.header.stamp.nsecs
        if current_time is not self.prev_t:
            x_dot = 1000000000*(com_mark.pose.position.x - self.prev_x)/(current_time - self.prev_t)
            y_dot = 1000000000*(com_mark.pose.position.y - self.prev_y)/(current_time - self.prev_t)

        x_cap = x_dot*sqrt(self.z_zero/self.g)
        y_cap = y_dot*sqrt(self.z_zero/self.g)

        # send CP position to RViZ
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.pose.position.x = x_cap
        self.marker.pose.position.y = y_cap
        self.marker.pose.position.z = 0
        rospy.logdebug("capture point is at " + str(self.marker.pose.position))

        self.prev_x = com_mark.pose.position.x
        self.prev_y = com_mark.pose.position.y
        self.prev_t = current_time

        return self.marker
