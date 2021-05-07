import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA


def generate_marker_msg(position, orientation):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = 'world_ned'
    marker.ns = 'marker'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.pose = Pose(position=Point(*position),
                       orientation=Quaternion(*orientation))
    marker.scale = Vector3(1, 1, 1)
    marker.color = ColorRGBA(.5, .5, .5, 1)
    return marker


def main():
    rospy.init_node('marked_pos_publisher')
    pub = rospy.Publisher('/sim/markers', Marker, queue_size=10)
    position = [1, 2, 3]
    orientation = [0, 0, 0, 1]
    marker = generate_marker_msg(position, orientation)
    while not rospy.is_shutdown():
        pub.publish(marker)
    print(marker)


if __name__ == '__main__':
    main()
