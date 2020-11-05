#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import inf
import math
import json

class ControlCenter:

    def __init__(self):
        print "registering publisher and subscriber"

        #reg subscriber, always callback when there is new number
        rospy.init_node('scan_values', anonymous=True)
        self.sub = rospy.Subscriber('/ifl_turtlebot1/scan', LaserScan, self.callback)

        #reg publisher , only when change pos() called, it is used 
        self.pub = rospy.Publisher("/ifl_turtlebot1/cmd_vel", Twist, queue_size=10)



    def callback(self,msg):
        
        self.ranges= msg.ranges
       
        self.range_max =msg.range_max
        stop_forward =msg.range_min
        print "min angle:",(msg.angle_min*180/math.pi), "max angle", (msg.angle_max*180/math.pi), "increment:",(msg.angle_increment*180/math.pi)

        self.dynamic_maneuver(self.ranges)
    
    
        


    def split(self, a, n):
        k, m = divmod(len(a), n)
        return (a[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(n))

    def uninf(self, a, max_val):
        if (a == inf):
            print("inf to ",  max_val)
            return max_val
        return a

    def dynamic_maneuver(self, ranges):
        
        MAX_RANGE = 3.5
        CONST_LIN = 0.6
        CONST_ANG = 0.1
        
        vl = self.uninf(ranges[30] , MAX_RANGE)
        vm = self.uninf(ranges[0]  , MAX_RANGE)
        vr = self.uninf(ranges[330], MAX_RANGE)

        print('vv----------------------------vv')
        print(vl , vm , vr)

        control_linear_vel  = vm / (2+abs(vl - vr))     * CONST_LIN
        control_angular_vel = (1 / vm) * (1-(vr/vl))    * CONST_ANG
        #annahme: rechtherum is neg                                                                                               left - right
        print("linVel: ", control_linear_vel)
        print("angVel: ", control_angular_vel)
        
        twist = Twist()
        twist.linear.x = control_linear_vel
        twist.angular.z = control_angular_vel
        self.pub.publish(twist)



if __name__=="__main__":


    try: 
        start=ControlCenter()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
