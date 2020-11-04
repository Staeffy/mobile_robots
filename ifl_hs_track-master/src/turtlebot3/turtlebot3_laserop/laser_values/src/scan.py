#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print len(msg.ranges)
    
    ranges= msg.ranges
    max_value = max(ranges)
    max_index =ranges.index(max_value)
    print max_index


    right_angle= msg.ranges[270]
    left_angle = msg.ranges[90]


    if (right_angle < left_angle):
        print "right side approaching wall"

    if (right_angle > left_angle):
        print "left side approaching wall"

    if (right_angle == left_angle): 
        print "driving perfectly in the middle"
        # values at 0 degree
    #print msg.ranges[0]
    # values at 90 degree
    #print msg.ranges[90]
    # values at 180 degree
    #print msg.ranges[180]
    

rospy.init_node('scan_values')
sub = rospy.Subscriber('/ifl_turtlebot1/scan', LaserScan, callback)
rospy.spin()