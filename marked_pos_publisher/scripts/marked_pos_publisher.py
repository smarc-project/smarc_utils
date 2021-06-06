#!/usr/bin/env python
import re
import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler


def get_ros_param(name):
    """Try to get rosparam from parameter server, shutdown the node if fails."""
    try:
        value = rospy.get_param(name)
    except KeyError:
        print('Parameter %s not found. Shutting down node...' % name)
        sys.exit(1)
    return value


def xml_string_param_to_list(string_param):
    """Given a list string param in XML, convert it to a list of float.
    Used to parse position and orientation of objects.
    """
    return [float(x) for x in string_param.split(' ')]


class MarkedPosPublisher:
    """Publish objects in /stonefish_simulator/marked_positions
    as MarkerArray (can be visualized in RViz)"""
    def __init__(self, topic_name, marked_position_param, mesh_param):
        self.pub = rospy.Publisher(topic_name, MarkerArray, queue_size=2)
        self.marked_positions = get_ros_param(marked_position_param)
        self.mesh_xml_paths = get_ros_param(mesh_param)
        self.marker_array = self.generate_marker_array_msg()

    def parse_marker_pose(self, marker_info):
        """Parse position and orientation from marker_info, convert coordinates
        from stonefish to RViz and return a Pose message"""
        # XY positions are reversed in stonefish and rviz
        position = xml_string_param_to_list(marker_info['position'])
        position[0], position[1] = position[1], position[0]
        # Stonefish's z-axis points downwards
        position[2] = -position[2]

        orientation = xml_string_param_to_list(marker_info['orientation'])
        quaternion = quaternion_from_euler(*orientation)
        pose = Pose(position=Point(*position),
                    orientation=Quaternion(*quaternion))
        return pose

    def parse_marker_mesh(self, marker_info):
        try:
            mesh_xml_path = self.mesh_xml_paths[marker_info['mesh']]
            return 'file://%s' % mesh_xml_path
        except KeyError:
            print('Error: mesh at %s not found' % (marker_info['mesh']))

    def parse_marker_color(self, marker_info):
        colors = {'red': (1, 0, 0, 1), 'white': (1, 1, 1, 1)}
        color = colors[marker_info['color']]
        return ColorRGBA(*color)

    def generate_marker_msg(self, marker_name, marker_info):
        """Given marker_name and marker_info, return a Marker message"""
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = marker_name

        marker.pose = self.parse_marker_pose(marker_info)
        marker.color = self.parse_marker_color(marker_info)
        marker.scale = Vector3(1, 1, 1)

        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = self.parse_marker_mesh(marker_info)

        return marker

    def generate_marker_array_msg(self):
        """Go through all marked_positions and generate a MarkerArray message"""
        marker_array = MarkerArray()
        for marker_name, marker_info in self.marked_positions.items():
            marker = self.generate_marker_msg(marker_name, marker_info)
            marker_array.markers.append(marker)
        return marker_array

    def publish_marker_array_msg(self):
        self.pub.publish(self.marker_array)


def main():
    rospy.init_node('marked_pos_publisher')
    rospy.Rate(1)

    robot_name_param = '~robot_name'
    if rospy.has_param(robot_name_param):
        robot_name = rospy.get_param(robot_name_param)
        print('Getting robot_name = {} from param server'.format(robot_name))
    else:
        robot_name = 'sam'
        print('{} param not found in param server.\n'
              'Setting robot_name = {} (default value).'.format(
                  robot_name_param, robot_name))

    topic_name = '/{}/sim/marked_positions'.format(robot_name)
    marked_position_param = '/stonefish_simulator/marked_positions'
    mesh_param = '/stonefish_simulator/mesh'

    marked_pos_pub = MarkedPosPublisher(topic_name, marked_position_param,
                                        mesh_param)

    while not rospy.is_shutdown():
        marked_pos_pub.publish_marker_array_msg()


if __name__ == '__main__':
    main()
