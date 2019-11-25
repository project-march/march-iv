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
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from visualization_msgs.msg import Marker


class CoMCalculator(object):

    def __init__(self, robot, tf_buffer):
        self.tf_buffer = tf_buffer

        self.links = dict(filter(lambda (_, l): l.inertial is not None, robot.link_map.items()))
        self.mass = sum(l.inertial.mass for (_, l) in self.links.items())

        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

    def calculate_com(self):
        x = 0
        y = 0
        z = 0
        for link in self.links:
            try:
                trans = self.tf_buffer.lookup_transform("world", link, rospy.Time())

                to_transform = geometry_msgs.msg.PointStamped()
                to_transform.point.x = self.links[link].inertial.origin.xyz[0]
                to_transform.point.y = self.links[link].inertial.origin.xyz[1]
                to_transform.point.z = self.links[link].inertial.origin.xyz[2]
                to_transform.header.frame_id = link
                to_transform.header.stamp = rospy.get_rostime()
                transformed = tf_geo.do_transform_point(to_transform, trans)

                # calculate part of CoM equation depending on link
                x += self.links[link].inertial.mass * transformed.point.x
                y += self.links[link].inertial.mass * transformed.point.y
                z += self.links[link].inertial.mass * transformed.point.z
            except tf2_ros.TransformException as err:
                rospy.logwarn("error in CoM calculation" + str(err))

        x = x/self.mass
        y = y/self.mass
        z = z/self.mass

        # send CoM position to RViZ
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        rospy.logdebug("center of mass is at " + str(self.marker.pose.position))

        return self.marker
