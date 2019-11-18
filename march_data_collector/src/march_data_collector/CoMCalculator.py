import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker


class CoMCalculator(object):

    def __init__(self, robot, tf_listener, tf_buffer):
        self.mass = 0
        self.links = robot.link_map
        self.tf_buffer = tf_buffer

        unnecessary_links = []
        for link in self.links:
            if self.links[link].inertial is None:
                unnecessary_links.append(link)

        for link in unnecessary_links:
            del self.links[link]

        for link in self.links:
            self.mass += self.links[link].inertial.mass

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
                rospy.logwarn("error in CoM calculation")

        x = x/self.mass
        y = y/self.mass
        z = z/self.mass

        # send CoM position to RViZ
        self.marker.header.stamp = rospy.Time()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        rospy.logdebug("center of mass is at " + str(self.marker.pose.position))

        return self.marker
