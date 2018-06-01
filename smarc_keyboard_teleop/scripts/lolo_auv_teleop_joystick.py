#!/usr/bin/python

import pygame
from pygame.constants import K_LEFT, K_RIGHT, K_UP, K_DOWN, K_w, K_s
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Joy

class TeleopServer(object):

    def callback(self, image_msg):

	try:
	    cv_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
	except CvBridgeError as e:
	    print(e)

	self.surface = pygame.image.frombuffer(cv_image.tostring(), cv_image.shape[:2], "RGB")

    def joy_callback(self, data):

	fin_angle = 60
        thrust_level = 200.

	fin0angle = 0. # top
	fin1angle = 0. # left
	fin2angle = 0. # down
	fin3angle = 0. # right
	fin4angle = 0. # down
	fin5angle = 0. # right
	backfinangle = 0. # right
	thrust = 0.

	fin0angle = fin_angle*data.axes[0]
	fin1angle = fin_angle*data.axes[0]
	fin2angle = -fin_angle*data.axes[0]
	fin3angle = -fin_angle*data.axes[0]
	
	fin4angle = fin_angle*data.axes[1]
	fin5angle = -fin_angle*data.axes[1]
	backfinangle = -fin_angle*data.axes[1]
	header = Header()

	self.thruster0.publish(header, thrust_level*data.axes[4])
	self.thruster1.publish(header, thrust_level*data.axes[4])
    
	self.fin0.publish(header, fin0angle)
	self.fin1.publish(header, fin1angle)
	self.fin2.publish(header, fin2angle)
	self.fin3.publish(header, fin3angle)
	self.fin4.publish(header, fin4angle)
	self.fin5.publish(header, fin5angle)
	self.backfin.publish(header, backfinangle)

    def __init__(self):
	
	rospy.init_node('keyboard_teleop', anonymous=True)
	pygame.init()
	
        self.surface = None
        self.bridge = CvBridge()
        self.auv_name = rospy.get_param(rospy.get_name() + '/auv_instance', 'lolo_auv')

	self.thruster0 = rospy.Publisher(self.auv_name + '/thrusters/0/input', FloatStamped, queue_size=10)
	self.thruster1 = rospy.Publisher(self.auv_name + '/thrusters/1/input', FloatStamped, queue_size=10)
	self.fin0 = rospy.Publisher(self.auv_name + '/fins/1/input', FloatStamped, queue_size=10)
	self.fin1 = rospy.Publisher(self.auv_name + '/fins/0/input', FloatStamped, queue_size=10)
	self.fin2 = rospy.Publisher(self.auv_name + '/fins/2/input', FloatStamped, queue_size=10)
	self.fin3 = rospy.Publisher(self.auv_name + '/fins/3/input', FloatStamped, queue_size=10)
	self.fin4 = rospy.Publisher(self.auv_name + '/fins/4/input', FloatStamped, queue_size=10)
	self.fin5 = rospy.Publisher(self.auv_name + '/fins/5/input', FloatStamped, queue_size=10)
	self.backfin = rospy.Publisher(self.auv_name + '/back_fins/0/input', FloatStamped, queue_size=10)

	rospy.Subscriber(self.auv_name + "/" + self.auv_name + "/camera_thruster/camera_image", Image, self.callback)
	rospy.Subscriber("/joy", Joy, self.joy_callback)

	screen = pygame.display.set_mode((200, 200))
	pygame.display.flip()

	clock = pygame.time.Clock()
	while not rospy.is_shutdown():
	    if self.surface is not None:
		screen.blit(self.surface, (0, 0))
	    pygame.display.update()
	    clock.tick(10)
	    pygame.event.pump()

if __name__ == "__main__":
    
    teleop = TeleopServer()
