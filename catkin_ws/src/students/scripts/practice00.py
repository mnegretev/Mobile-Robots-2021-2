#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
# PRACTICE 0 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot forward until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist
import numpy

NAME = "Rojas Mosqueda Axel Javier"

class Robot:
    
    
    
    def __init__(self,lecturas):
        self.lecturas_escaner=lecturas
        
        
    def callback_laser_scan(self,msg):
        self.lecturas_escaner=list(msg.ranges)
        
       
    def main(self):
        
        print "PRACTICE 00 - " + NAME
        rospy.init_node("practice00")
        rospy.Subscriber("/scan", LaserScan, self.callback_laser_scan)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        loop = rospy.Rate(10)
        self.twist=Twist()
        while not rospy.is_shutdown():
            if len(self.lecturas_escaner)>0:
                if self.lecturas_escaner[len(self.lecturas_escaner)//2]>1.0:
                    self.twist.linear.x=0.5
                elif self.lecturas_escaner[len(self.lecturas_escaner)//2]<=1.0:
                    self.twist.linear.x=0
                    
                
            
            self. pub_cmd_vel.publish(self.twist)    
            loop.sleep()


if __name__ == '__main__':
    try:
        Robot1=Robot([])
        Robot1.main()
    except rospy.ROSInterruptException:
        pass
    
