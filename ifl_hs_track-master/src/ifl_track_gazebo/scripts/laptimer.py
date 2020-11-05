#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Range

class Laptimer(object):    

    def __init__(self, ns):
        self.ns = ns
        self.start_sec = 0
        self.start_nsec = 0
        self.detected = False
        self.lis = rospy.Subscriber(str(self.ns+"detector"), Range, self.callback)
    	print("laptimer initialized")


    def callback(self, data):
        
        if data.range < 0.87:
            if self.detected == False:
                self.timer()
            self.detected = True
        else:    
            self.detected = False
    
    def timer(self):
        now = rospy.get_rostime()
        if self.start_nsec > 0:
            mins = int((now.secs-self.start_sec)/60)
            secs = (now.secs-self.start_sec)%60
            rospy.loginfo("laptime: %d:%d:%d" % (mins, secs, now.nsecs/1000000))
        self.start_sec = now.secs
        self.start_nsec = now.nsecs
        rospy.loginfo("laptimer started")
        
if __name__ == '__main__':
    rospy.init_node('laptimer')
    ns = ""
    try:
        ns = rospy.get_param('~ns')
    except:
        pass
    server = Laptimer(ns)
    rospy.spin()
