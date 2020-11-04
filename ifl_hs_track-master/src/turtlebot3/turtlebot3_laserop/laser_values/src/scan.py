#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


#set max velocity

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)



"""get distance from robot for left and right side """
global status
status=0

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
        status=1

    if (right_angle > left_angle):
        print "left side approaching wall"
        status=2

    if (right_angle == left_angle): 
        print "driving perfectly in the middle"
        status =0

    
        # values at 0 degree
    #print msg.ranges[0]
    # values at 90 degree
    #print msg.ranges[90]
    # values at 180 degree
    #print msg.ranges[180]
    


if __name__=="__main__":

    #1. get current stats
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/ifl_turtlebot1/scan', LaserScan, callback)
    rospy.spin()

    print "curent status =", status

    if status == 1:
        target_angular_vel=+1
    
    if status == 2:
        target_angular_vel=-1

    if status == 0:
        target_angular_vel=0.0

    #2. check ranges 

    #3. update velocity 
    rospy.init_node("turtlebot3_teleop")
    pub = rospy.Publisher("/ifl_turtlebot1/cmd_vel", Twist, queue_size=10)

    #status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.1
    control_angular_vel = 0.0



    twist = Twist()

    #control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    #control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    pub.publish(twist)