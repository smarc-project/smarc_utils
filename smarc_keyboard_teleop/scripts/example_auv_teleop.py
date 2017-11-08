#!/usr/bin/python

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

	thruster = rospy.Publisher('/example_auv/thrusters/0/input', FloatStamped, queue_size=10)
	fin0 = rospy.Publisher('/example_auv/fins/0/input', FloatStamped, queue_size=10)
	fin1 = rospy.Publisher('/example_auv/fins/1/input', FloatStamped, queue_size=10)
	fin2 = rospy.Publisher('/example_auv/fins/2/input', FloatStamped, queue_size=10)
	fin3 = rospy.Publisher('/example_auv/fins/3/input', FloatStamped, queue_size=10)

	rospy.Subscriber("/example_auv/example_auv/camera_thruster/camera_image", Image, self.callback)

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
            thrust = 0.

	    if keys[K_LEFT]:
		fin0angle = -fin_angle
		fin2angle = fin_angle
	    if keys[K_RIGHT]:
		fin0angle = fin_angle
		fin2angle = -fin_angle
	    if keys[K_UP]:
		fin1angle = fin_angle
		fin3angle = -fin_angle
	    if keys[K_DOWN]:
		fin1angle = -fin_angle
		fin3angle = fin_angle
	    if keys[K_w]:
                thruster.publish(header, thrust_level)
            if keys[K_s]:
                thruster.publish(header, 0.)
	
	    fin0.publish(header, fin0angle)
	    fin1.publish(header, fin1angle)
	    fin2.publish(header, fin2angle)
	    fin3.publish(header, fin3angle)

	    pygame.event.pump()
	    clock.tick(10)

if __name__ == "__main__":
    
    teleop = TeleopServer()
