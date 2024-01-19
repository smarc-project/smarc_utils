#!/usr/bin/python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

from aruco_msgs.msg import MarkerArray

class VehicleDR(object):

    def __init__(self):

        self.makers_topic = rospy.get_param('~markers_topic', '/aruco_marker_publisher/markers')
        self.camera_frame = rospy.get_param('~base_frame_2d', 'cm_station/uw_camera')
        self.cm_base_frame = rospy.get_param('~base_frame_2d', 'cm_station/uw_camera')
        
        self.listener = tf.TransformListener()
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.tf_bc = tf2_ros.TransformBroadcaster()
        self.br = tf.TransformBroadcaster()
        self.transformStamped = TransformStamped()

        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
        
        self.thrust_cmd_sub = rospy.Subscriber(self.makers_topic, MarkerArray, self.markers_cb)

        self.ds_position = None
        self.auv_position = None

        rospy.spin()

    # 102 back port
    # 107 back starboard
    def f(self, x):
        return {
            110: "sam/qr_link_optical_0",
            103: "sam/qr_link_1",
            108: "dc_station/base_link",
            109: "test",
        }.get(x, None)  


    def markers_cb(self, markers_msg):

        for marker in markers_msg.markers:

            if self.f(marker.id) is not None:   
                self.transformStamped.transform.translation.x = marker.pose.pose.position.x
                self.transformStamped.transform.translation.y = marker.pose.pose.position.y
                self.transformStamped.transform.translation.z = marker.pose.pose.position.z
                self.transformStamped.transform.rotation = marker.pose.pose.orientation
                self.transformStamped.header.frame_id = "cm_station/base_link"
                self.transformStamped.child_frame_id = self.f(marker.id)
                self.transformStamped.header.stamp = rospy.Time.now()
                self.tf_bc.sendTransform(self.transformStamped)

                if self.f(marker.id) == "dc_station/base_link":
                    self.ds_position = np.array([marker.pose.pose.position.x,
                                                 marker.pose.pose.position.y,
                                                 marker.pose.pose.position.z])
                
                if self.f(marker.id) == "sam/qr_link_optical_0":
                    self.auv_position = np.array([marker.pose.pose.position.x,
                                                 marker.pose.pose.position.y,
                                                 marker.pose.pose.position.z])
                    print("AUV detecteeeeeeeeeeeeeeeeeeed")
                
                    if self.auv_position.any() and self.ds_position.any():
                        print("Distance DS to AUV {0}".format(np.linalg.norm(self.ds_position-self.auv_position)))


if __name__ == "__main__":
    rospy.init_node('cams_detections')
    try:
        VehicleDR()
    except rospy.ROSInterruptException:
        pass
