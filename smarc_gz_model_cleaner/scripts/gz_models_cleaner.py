#!/usr/bin/env python

# Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
