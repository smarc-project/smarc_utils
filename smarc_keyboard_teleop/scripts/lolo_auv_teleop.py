#!/usr/bin/python

# Copyright 2018 Nils Bore (nbore@kth.se)
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
from pygame.constants import K_LEFT, K_RIGHT, K_UP, K_DOWN, K_w, K_s
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TeleopServer(object):

    def callback(self, image_msg):

	try:
	    cv_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
	except CvBridgeError as e:
	    print(e)

	self.surface = pygame.image.frombuffer(cv_image.tostring(), cv_image.shape[:2], "RGB")

    def __init__(self):
	
	rospy.init_node('keyboard_teleop', anonymous=True)
	pygame.init()
	
        self.surface = None
        self.bridge = CvBridge()
        self.auv_name = rospy.get_param(rospy.get_name() + '/auv_instance', 'lolo_auv')

	thruster0 = rospy.Publisher(self.auv_name + '/thrusters/0/input', FloatStamped, queue_size=10)
	thruster1 = rospy.Publisher(self.auv_name + '/thrusters/1/input', FloatStamped, queue_size=10)
	fin0 = rospy.Publisher(self.auv_name + '/fins/1/input', FloatStamped, queue_size=10)
	fin1 = rospy.Publisher(self.auv_name + '/fins/0/input', FloatStamped, queue_size=10)
	fin2 = rospy.Publisher(self.auv_name + '/fins/2/input', FloatStamped, queue_size=10)
	fin3 = rospy.Publisher(self.auv_name + '/fins/3/input', FloatStamped, queue_size=10)
	fin4 = rospy.Publisher(self.auv_name + '/fins/4/input', FloatStamped, queue_size=10)
	fin5 = rospy.Publisher(self.auv_name + '/fins/5/input', FloatStamped, queue_size=10)
	backfin = rospy.Publisher(self.auv_name + '/back_fins/0/input', FloatStamped, queue_size=10)

	rospy.Subscriber(self.auv_name + "/" + self.auv_name + "/camera_thruster/camera_image", Image, self.callback)

	screen = pygame.display.set_mode((200, 200))
	pygame.display.flip()
	header = Header()

	fin_angle = 60
        thrust_level = 200.

	clock = pygame.time.Clock()
	while not rospy.is_shutdown():

	    if self.surface is not None:
		screen.blit(self.surface, (0, 0))
	    pygame.display.update()

	    keys = pygame.key.get_pressed()
	    fin0angle = 0. # top
	    fin1angle = 0. # left
	    fin2angle = 0. # down
	    fin3angle = 0. # right
	    fin4angle = 0. # down
	    fin5angle = 0. # right
	    backfinangle = 0. # right
            thrust = 0.

	    if keys[K_LEFT]:
		fin0angle = -fin_angle
		fin1angle = -fin_angle
		fin2angle = fin_angle
		fin3angle = fin_angle
	    if keys[K_RIGHT]:
		fin0angle = fin_angle
		fin1angle = fin_angle
		fin2angle = -fin_angle
		fin3angle = -fin_angle
	    if keys[K_UP]:
		fin4angle = fin_angle
		fin5angle = -fin_angle
		backfinangle = -fin_angle
	    if keys[K_DOWN]:
		fin4angle = -fin_angle
		fin5angle = fin_angle
		backfinangle = fin_angle
	    if keys[K_w]:
                thruster0.publish(header, thrust_level)
                thruster1.publish(header, thrust_level)
            if keys[K_s]:
                thruster0.publish(header, 0.)
                thruster1.publish(header, 0.)
	
	    fin0.publish(header, fin0angle)
	    fin1.publish(header, fin1angle)
	    fin2.publish(header, fin2angle)
	    fin3.publish(header, fin3angle)
	    fin4.publish(header, fin4angle)
	    fin5.publish(header, fin5angle)
            backfin.publish(header, backfinangle)
	    pygame.event.pump()
	    clock.tick(10)

if __name__ == "__main__":
    
    teleop = TeleopServer()
