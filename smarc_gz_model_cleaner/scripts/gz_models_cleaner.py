#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest 

class ModelsCleaner(object):
    def __init__(self):

   		rospy.loginfo("Gazebo_models_cleaner launched")
		self.node_name = "gazebo_models_handler"
		self.auv_name = rospy.get_param(rospy.get_name() + '/auv_instance', 'lolo_auv')

		# Wait for service providers
		self.dlt_model_srv = "/gazebo/delete_model"
		rospy.wait_for_service(self.dlt_model_srv, timeout=30)
		self.delete_model_srv = rospy.ServiceProxy(self.dlt_model_srv, DeleteModel)
		rospy.loginfo("%s: Services received!", self.node_name)

		rospy.on_shutdown(self.cleanup_cb)
		rospy.spin()

    def cleanup_cb(self):
		rospy.loginfo("%s: Calling gazebo delete_models", self.node_name)

  		try:
			delete_robot = DeleteModelRequest(self.auv_name)
			delete_model_res = self.delete_model_srv(delete_robot)
		
		except rospy.ServiceException, e:
			print "Service call to gazebo delete_models failed: %s"%e


if __name__ == "__main__":
    
    rospy.init_node('gazebo_models_cleaner')
    try:
        ModelsCleaner()

    except rospy.ROSInterruptException:
        pass