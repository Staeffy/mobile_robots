#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
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

        self.maneuver(ranges, max_forward,stop_forward)



    def maneuver(self,ranges,max_forward,stop_forward):

        right_side_range=ranges[270]
        #right_side_range=("%.2f" % right_side_range)
        left_side_range =ranges[90]
        #left_side_range=("%.2f" % left_side_range)

        forward_range =ranges[0]
        #forward_range=("%.2f" % forward_range)

        max_forward=("%.2f" % max_forward)
        stop_forward=("%.2f" % stop_forward)

        print "current ranges", "r",right_side_range, "l:", left_side_range, "f", forward_range, "max_f", max_forward

        if  forward_range <0.50:
            print "fw_range=",forward_range, "stop moving forward"
            status=5
            
        if forward_range >=0.5:
            status=0
            print "fw range =",forward_range, "ready to go forward"

        if right_side_range < 0.40:
            status=1
            print "right side approaching wall"

        if left_side_range <0.40:
            print "left side approaching wall"
            status=2
            
        else:
            print"idk what to do"
      
        print "eval status", status
        
        #reaktion auf status
        self.change_position(status)

    def change_position(self,status):
  
     
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        #print("received status", status)
        #if self.status ==3:
            #control_linear_vel=0.1
            #control_angular_vel=0.0
            #print ""

        if  status == 1:
            control_angular_vel=0.1
            #control_linear_vel=0.1
            print "turn left"
        
        if  status == 2:
            control_angular_vel=-0.1
            #control_linear_vel=0.1
            print "turn right"

        if  status == 0:
            control_linear_vel=+0.1
            control_angular_vel=0.0
            print "speed up"
        
        if  status ==5:
            control_linear_vel=0.0
            print "stop moving"
       
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
