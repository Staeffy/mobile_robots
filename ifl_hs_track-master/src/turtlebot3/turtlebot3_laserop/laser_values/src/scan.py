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
       
        #max_forward =msg.range_max
        #stop_forward =msg.range_min
        #print "getting raw range 270",ranges[270]
        #print "min angle:",(msg.angle_min*180/math.pi), "max angle", (msg.angle_max*180/math.pi), "increment:",(msg.angle_increment*180/math.pi)

            #to improve=zahlen runden
        #werte analysieren, dann reagieren

        self.maneuver(ranges)



    def maneuver(self,ranges):

        right_side_range=ranges[270]
        right_diag_range=ranges[315]
        #right_side_range=("%.2f" % right_side_range)
        left_side_range =ranges[90]
        left_diag_range =ranges[45]
        #left_side_range=("%.2f" % left_side_range)

        forward_range =ranges[0]
        #forward_range=("%.2f" % forward_range)

        #max_forward=("%.2f" % max_forward)
       # stop_forward=("%.2f" % stop_forward)

        print "current ranges", "r",right_side_range, "rd", right_diag_range, "l:", left_side_range,"ld",left_diag_range, "f", forward_range, 

        if  forward_range <0.50:
            print "fw_range=",forward_range, "stop moving forward"
            Forward=False
            
        if forward_range >=0.5:
            print "fw range =",forward_range, "ready to go forward"
            Forward=True

        if right_side_range < 0.30:
            Right=False
            print "right side approaching wall"
        
        if right_diag_range < 0.5:
            Right_d=False
            print "diag right side getting close"

        if right_diag_range > 0.5:
            print "diag right side has space"
            Right_d=True

        if right_side_range > 0.30:
            Right=True
            print "right side has space"

        if left_side_range <0.30:
            print "left side approaching wall"
            Left=False

        if left_side_range >0.30:
            print "left side approaching wall"
            Left=True

        if left_diag_range < 0.50:
            Left_d=False
            print "diag left side getting close"

        if left_diag_range > 0.50:
            print "diag left side has space"
            Left_d=True


        if (left_diag_range > right_diag_range):
            print "more space on left side"
            status=1
        
        if (right_diag_range> left_diag_range):
            print "more space on right side"
            status=2

            
        else:
            print"idk what to do"
      
        print "eval status",status,  "Left",Left , "Right", Right, "Forward", Forward
        
        #reaktion auf status
        self.change_position(Left, Right, Forward,Right_d, Left_d,status)

    def change_position(self,Left,Right,Left_d,Right_d,Forward,status):
  
     
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

       

        if  Right == False:
            control_angular_vel=+0.2
            #control_linear_vel=0.1
            print "turning left"
        
        if Right_d == False:
            control_angular_vel=+0.1
            control_linear_vel=+0.1
            print "adjusting left, right side getting close"


        if Left_d == False:
            control_angular_vel=-0.1
            control_linear_vel=+0.1

            print "adjusting right, right left getting close"

        if Left_d ==True and Right_d==True:
            control_angular_vel=0.0
            control_linear_vel=0.3
            print"right and left diag have space"
        
        if  Left == False:
            control_angular_vel=-0.2
            #control_linear_vel=0.1
            print "turning right"

        if  Forward == True and Right_d ==True and Left_d==True:
            control_linear_vel=+0.3
            control_angular_vel=0.0
            #control_angular_vel=0.0
            print "all clear,speed up"

        if Forward ==True and status==1:
            print "forward clear,adjust left"
            control_angular_vel=0.2
            control_linear_vel=+0.1
        
        if Forward ==True and status==2:
            print"forward clear, adjust right"
            control_linear_vel=0.2
            control_angular_vel=-0.1

        
        if  Forward ==False and status==1:
            control_linear_vel=0.0
            control_angular_vel=+0.1
            print "stop moving + turn left"

        if  Forward ==False and status==2:
            control_linear_vel=0.0
            control_angular_vel=-0.1
            print "stop moving + turn right"

            
       

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
