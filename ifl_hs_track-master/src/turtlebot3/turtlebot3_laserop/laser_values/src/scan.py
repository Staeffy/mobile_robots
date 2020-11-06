#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
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
        
        ranges= msg.ranges
       
        self.range_max =msg.range_max
        stop_forward =msg.range_min
        print "min angle:",(msg.angle_min*180/math.pi), "max angle", (msg.angle_max*180/math.pi), "increment:",(msg.angle_increment*180/math.pi)

        self.dynamic_maneuver(ranges)
    
    
        


    def split(self, a, n):
        k, m = divmod(len(a), n)
        return (a[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(n))

    def uninf(self, a, max_val):
        if (a == inf):
            print("inf to ",  max_val)
            return max_val
        return a

    def dynamic_maneuver(self, ranges):
    
        INF_RANGE = 3.5
        CONST_LIN = 0.4
        CONST_ANG = 0.07
        FUNNEL_DEG = 45
        FRONT_RATIO_DEGREE = 1.5
        
        ranges = np.array(ranges)
        ranges[ranges == np.inf] = 3.5

        #for n, i in enumerate(ranges):
        #    if i == inf:
        #        ranges[n] = 3


        # Robo Front Ranges
        vl = ranges[FUNNEL_DEG]   #self.uninf(ranges[30] , INF_RANGE)
        vm = ranges[0]    #self.uninf(ranges[0]  , INF_RANGE)
        vr = ranges[360-FUNNEL_DEG]  #self.uninf(ranges[330], INF_RANGE)

        # Robo max View Ranges
         #frontRanges = ranges[:]
         #del frontRanges[90:270]
        right_ranges = ranges[:90]
        left_ranges = ranges[-90::]

        sum_right = np.sum(right_ranges)
        sum_left =  np.sum(left_ranges)
        
        if  sum_right < sum_left :
            print("Left Curve")
            print(left_ranges)
            maxRange = np.max(left_ranges)
            d_vm = 359-np.where(left_ranges==maxRange)[0][0]
        else:
            print("Right Curve")
            print(right_ranges)
            maxRange = np.max(right_ranges)
            d_vm = np.where(right_ranges==maxRange)[0][0]

        print("d_vm",d_vm)
        d_vr = (d_vm - FUNNEL_DEG) % 360
        d_vl = (d_vm + FUNNEL_DEG) % 360

        dvm = ranges[d_vm]
        dvr = ranges[d_vr]
        dvl = ranges[d_vl]

        print('vv----------------------------vv')
        print(FUNNEL_DEG," deg" , " 0 deg " , 360-FUNNEL_DEG," deg ")
        print(vl , vm , vr)
        print(d_vl," deg " , d_vm," deg " , d_vr," deg ")
        print(dvl, dvm , dvr)


        front_linear_vel  = vm / (2+abs(vl - vr))     * CONST_LIN
        front_angular_vel = (1 / vm) * (1-(vr/vl))    * CONST_ANG
    
        degreed_linear_vel  = dvm / (2+abs(dvl - dvr))     * CONST_LIN
        degreed_angular_vel = (1 / dvm) * (1-(dvr/dvl))    * CONST_ANG

        #annahme: rechtherum is neg left - right

        control_linear_vel  = (FRONT_RATIO_DEGREE * front_linear_vel )  + ((1/FRONT_RATIO_DEGREE) * degreed_linear_vel  )
        control_angular_vel = (FRONT_RATIO_DEGREE * front_angular_vel)  + ((1/FRONT_RATIO_DEGREE) * degreed_angular_vel )



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
