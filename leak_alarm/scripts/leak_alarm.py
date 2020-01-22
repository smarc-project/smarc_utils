#!/usr/bin/python

import rospy
import os
from sam_msgs.msg import Leak 

class LeakAlarm(object):

    def pulse(self, event):
        rospy.loginfo("Checking for leaks %s", rospy.get_time())

    def leak_cb(self, leak_msg):
        if leak_msg.value:
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                os.system('espeak -a 200 -p 99 -ven "Leak"')
                r.sleep()

    def __init__(self):

        self.leak_subs = rospy.Subscriber('/sam/core/leak_fb', Leak, self.leak_cb)
        self.timer = rospy.Timer(rospy.Duration(5.), self.pulse)
        
        rospy.spin()

if __name__ == "__main__":
    
    rospy.init_node("leak_alarm")
    try:
        leak_alarm = LeakAlarm()
    except rospy.ROSInterruptException:
        pass
        
