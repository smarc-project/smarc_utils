#!/usr/bin/python3

import rospy
import os
from sam_msgs.msg import Leak 
from smarc_bt.msg import MissionControl

class LeakAlarm(object):

    def pulse(self, event):
        rospy.loginfo("Checking for leaks %s", rospy.get_time())

    def fb(self, x):
        return {
            0 : 'RUNNING',
            1 : 'STOPPED',
            2 : 'PAUSED',
            3 : 'EMERGENCY',
            4 : 'RECEIVED',
            5 : 'COMPLETED',
        }[x]

    def mission_control_cb(self, fb_msg):
        word = self.fb(fb_msg.plan_state)
        os.system('espeak -a 200 -p 99 -ven ' + word)

    def __init__(self):

        self.mc_subs = rospy.Subscriber('/sam/smarc_bt/mission_control', MissionControl, self.mission_control_cb)
        # self.timer = rospy.Timer(rospy.Duration(5.), self.pulse)
        
        rospy.spin()

if __name__ == "__main__":
    
    rospy.init_node("leak_alarm")
    try:
        leak_alarm = LeakAlarm()
    except rospy.ROSInterruptException:
        pass
        
