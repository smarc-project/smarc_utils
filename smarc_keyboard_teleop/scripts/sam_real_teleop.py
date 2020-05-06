#!/usr/bin/python

# Copyright 2019 Ignacio Torroba (torroba@kth.se)
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

import pygame
from pygame.constants import K_LEFT, K_RIGHT, K_UP, K_DOWN, K_w, K_s, K_z, K_a, K_d, K_m, K_n
import rospy
import numpy as np
# from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header, Float64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sam_msgs.msg import ThrusterRPMs, ThrusterAngles, PercentStamped

class SAMTeleopServer(object):

	def vbsCB(self, vbs_status_msg):
		self.vbs_status = vbs_status_msg.value

	def callback(self, image_msg):
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
		except CvBridgeError as e:
		    print(e)

		self.surface = pygame.image.frombuffer(cv_image.tostring(), cv_image.shape[:2], "RGB")

	def __init__(self):
	
		pygame.init()
		
		# self.surface = None
		# self.bridge = CvBridge()
		# rospy.Subscriber("/sam_auv_1/sam_auv_1/camera_thruster/camera_image", Image, self.callback)

		self.thruster_top = rospy.get_param('~rpm_command', '/sam/core/rpm_cmd')
		self.vec_top = rospy.get_param('~vec_command', '/sam/core/thrust_vector_cmd')
		self.vbs_top = rospy.get_param('~vbs_command', '/sam/core/vbs_cmd')
		self.manual_top = rospy.get_param('~manual_top', '/manual_control_on')
		self.vbs_fb_top = rospy.get_param('~vbs_feedback', '/sam/core/vbs_fb')

		thruster_pub = rospy.Publisher(self.thruster_top, ThrusterRPMs, queue_size=10)
		vector_pub = rospy.Publisher(self.vec_top, ThrusterAngles, queue_size=10)
		vbs_pub = rospy.Publisher(self.vbs_top, PercentStamped, queue_size=10)
		manual_on_pub = rospy.Publisher(self.manual_top, Bool, queue_size=10)
		rospy.Subscriber(self.vbs_fb_top, PercentStamped, self.vbsCB)
		self.vbs_status = 50.0

        	screen = pygame.display.set_mode((200, 200))
        	pygame.display.flip()
        	header = Header()

        	rudder_angle = 0.12
        	altitude_angle = 0.12
        	thrust_level = 1000.
        	vbs_step = 10.0
        	manual_mode = False

        	clock = pygame.time.Clock()
        	while not rospy.is_shutdown():
        		vbs_abs = self.vbs_status

		  #   if self.surface is not None:
				# screen.blit(self.surface, (0, 0))
				# pygame.display.update()

			keys = pygame.key.get_pressed()
			steer = ThrusterAngles()
			thrust = ThrusterRPMs()
			vbs_msg = PercentStamped()

			if keys[K_m] or manual_mode:
				manual_mode = True
				thrust.thruster_1_rpm = 0.0
				thrust.thruster_2_rpm = 0.0
				steer.thruster_horizontal_radians = 0.0
				steer.thruster_vertical_radians = 0.0

				# Steering
				if keys[K_LEFT]:
					steer.thruster_horizontal_radians = -rudder_angle
				if keys[K_RIGHT]:
					steer.thruster_horizontal_radians = rudder_angle
				if keys[K_UP]:
					steer.thruster_vertical_radians = altitude_angle
				if keys[K_DOWN]:
					steer.thruster_vertical_radians = -altitude_angle
				vector_pub.publish(steer)

				# Thrusting
				if keys[K_w]:
					thrust.thruster_1_rpm = thrust_level
					thrust.thruster_2_rpm = thrust_level
					thruster_pub.publish(thrust)
				if keys[K_s]:
					thrust.thruster_1_rpm = -thrust_level
					thrust.thruster_2_rpm = -thrust_level
					thruster_pub.publish(thrust)

				# VBS
				#  if keys[K_a]:
					#  vbs_abs += vbs_step
					#  if vbs_abs > 100.0:
						#  vbs_abs = 100.0
					#  vbs_msg.value = vbs_abs
					#  vbs_pub.publish(vbs_msg)
				#  if keys[K_d]:
					#  vbs_abs -= vbs_step
					#  if vbs_abs < 0.0:
						#  vbs_abs = 0.0
					#  vbs_msg.value = vbs_abs
					#  vbs_pub.publish(vbs_msg)

			if keys[K_n]:
				manual_mode = False
				steer.thruster_horizontal_radians = 0.0
				steer.thruster_vertical_radians = 0.0
				vector_pub.publish(steer)

			manual_on_pub.publish(manual_mode)
			pygame.event.pump()
			clock.tick(10)

if __name__ == "__main__":

	rospy.init_node('sam_keyboard_teleop')

	try:
		SAMTeleopServer()
	except rospy.ROSInterruptException:
		pass
