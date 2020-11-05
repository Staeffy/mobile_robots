#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import json
import time

class ControlCenter:

    def __init__(self):
        print "registering publisher and subscriber"

        #reg subscriber, always callback when there is new number
        rospy.init_node('scan_values', anonymous=True)
        self.sub = rospy.Subscriber('/ifl_turtlebot1/scan', LaserScan, self.callback)

        #reg publisher , only when change pos() called, it is used 
        self.pub = rospy.Publisher("/ifl_turtlebot1/cmd_vel", Twist, queue_size=10)

        self.v = 1


    def callback(self,msg):
     
        ranges= msg.ranges
        print("\nGet Sensor Data: ")
        self.dynamic_maneuver(ranges)
        print("\n")
        time.sleep(2)


    
    def dynamic_maneuver(self, ranges):
        
        ranges = np.array(ranges)
        forward_space = ranges[0]


        if forward_space == np.inf:
            print("Go Straight: {}".format(self.v))
            twist = Twist()
            twist.linear.x = self.v
            twist.angular.z = 0
            self.pub.publish(twist)

            self.v = self.v + 1
            if self.v > 2:
                self.v = 2
            return None
        
        else: 
            median_angle = getRotationAngle(ranges)
            rotation_v = self.v * median_angle
            print("Rotation: {} / {} (Straigt/Rotation)".format(self.v, rotation_v))
            twist = Twist()
            twist.linear.x = self.v
            twist.angular.z = rotation_v
            self.pub.publish(twist)


    def getRotationAngle(self, ranges):

        ranges[ranges == np.inf] = 3.5
        right_ranges = ranges[:90]
        left_ranges = ranges[-90::]

        sum_right = np.sum(right_ranges)
        sum_left =  np.sum(left_ranges)
        
        print("Sum Right Ranges vs. Left Ranges ->  R:{} - L:{}".format(sum_right, sum_left))
        if  sum_right < sum_left :
            print("Left Curve")
            median_angle = 360 - np.median(np.where(left_ranges == 3.5)[0])
            print("Left Curve - Median Angle: {}".format(median_angle))
        else:
            print("Right Curve")
            median_angle = - np.median(np.where(right_ranges == 3.5)[0])
            print("Right Curve - Median Angle: {}".format(median_angle))

        
        return median_angle

if __name__=="__main__":


    try: 
        start=ControlCenter()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
