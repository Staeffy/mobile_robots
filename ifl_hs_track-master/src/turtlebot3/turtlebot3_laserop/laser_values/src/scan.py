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
     
        

        ranges= msg.ranges
       
        max_forward =msg.range_max
        stop_forward =msg.range_min
        print "getting raw range 270",ranges[270]
        print "min angle:",(msg.angle_min*180/math.pi), "max angle", (msg.angle_max*180/math.pi), "increment:",(msg.angle_increment*180/math.pi)

            #to improve=zahlen runden
        #werte analysieren, dann reagieren

        self.dynamic_maneuver(ranges)



    def split(self, a, n):
        k, m = divmod(len(a), n)
        return (a[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(n))

    def dynamic_maneuver(self, ranges):
        
        max_vel   = 0.4   #0,2    # max velocity of the robot if its directly in the middle
        LIN_CONST = 0.21  #0.1   # deaccelarating Factor if range values are unequal
        ANG_CONST = 0.15   #0,2    # ang accelerating if factors are unequal
        maxRange  = 3     #3
        CONST_Linear_Front_mid_ratio = 2

        rangeChunks = list(self.split(ranges, 6))

        range_left_front = rangeChunks[0]
        range_left_side  = rangeChunks[1]
        range_left_back  = rangeChunks[2]
        range_right_back = rangeChunks[3]
        range_right_side = rangeChunks[4]
        range_right_front= rangeChunks[5]

        avgRange_left_front = sum(range_left_front )/len(range_left_front )
        avgRange_left_side  = sum(range_left_side  )/len(range_left_side  )
        avgRange_left_back  = sum(range_left_back  )/len(range_left_back  )
        avgRange_right_back = sum(range_right_back )/len(range_right_back )
        avgRange_right_side = sum(range_right_side )/len(range_right_side )
        avgRange_right_front= sum(range_right_front)/len(range_right_front)

        if (avgRange_left_front == inf):
            avgRange_left_front = maxRange
        if (avgRange_right_front == inf):
            avgRange_right_front = maxRange
        if (avgRange_left_side == inf):
            avgRange_left_side = maxRange
        if (avgRange_right_side == inf):
            avgRange_right_side = maxRange
        if (avgRange_left_back == inf):
            avgRange_left_back = maxRange
        if (avgRange_right_back == inf):
            avgRange_right_back = maxRange

        print('vv----------------------------vv')
        print(avgRange_left_front , avgRange_right_front)
        print(avgRange_left_side  , avgRange_right_side )
        print(avgRange_left_back  , avgRange_right_back )
        print(avgRange_left_back  , avgRange_right_back )


        control_linear_vel  = max_vel - LIN_CONST * (CONST_Linear_Front_mid_ratio * abs (avgRange_right_front - avgRange_left_front) - (1/CONST_Linear_Front_mid_ratio) abs(avgRange_right_side - avgRange_left_side))
        control_angular_vel = ANG_CONST * ( ( (avgRange_left_front * avgRange_left_side) / (avgRange_right_front * avgRange_right_side) ) - ( (avgRange_right_front * avgRange_right_side) / (avgRange_left_front * avgRange_left_side) ) )
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
