#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
# PRACTICE 4 - POSITION CONTROL AND PATH TRACKING
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#

import sys
import rospy
import tf
import math
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetPlanRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

NAME = "GUZMAN_MARTINEZ"

pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    alpha=1
    beta=1
    v_max= 0.5
    w_max= 1
    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    error_a=math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a

    if(error_a <= -(math.pi)):
        error_a+= 2*math.pi
    else:
        error_a-= 2*math.pi
        
    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    
    cmd_vel.linear.x=v
    cmd_vel.linear.z=w
    
    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # Use the calculate_control function to move the robot along the path.
    # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
    # The publisher for the twist message is already declared as 'pub_cmd_vel'
    # You can use the following steps to perform the path tracking:
    #
    # Set local goal point as the first point of the path
    # Set global goal point as the last point of the path
    # Get robot position with [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    # Calculate global error as the magnitude of the vector from robot pose to global goal point
    # Calculate local  error as the magnitude of the vector from robot pose to local  goal point
    #
    # WHILE global error > tolerance  and not rospy.is_shutdown() #This keeps the program aware of signals such as Ctrl+C
    #     Calculate control signals v and w and publish the corresponding message
    #     loop.sleep()  #This is important to avoid an overconsumption of processing time
    #     Get robot position
    #     Calculate local error
    #     If local error is less than 0.3 (you can change this constant)
    #         Change local goal point to the next point in the path
    #     Calculate global error
    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    #
    idx=0
    [local_x, local_y]= path[idx]
    [global_x, global_y]= path[-1]
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    local_error = math.sqrt((local_x - robot_x)**2 + (local_y - robot_y)**2)
    while global_error > 0.1 and not rospy.is_shutdown():
        pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, local_x, local_y))
        loop.sleep()
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
        local_error = math.sqrt((local_x - robot_x)**2 + (local_y - robot_y)**2)
        if local_error < 0.3:
            idx+=1
            if idx >= len(path):
                idx= len(path) - 1
            [local_x, local_y] =path[idx]
        global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    pub_cmd_vel.publish(Twist())
    return
    
def callback_global_goal(msg):
    print ("Calculatin path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y]))
    clt_plan_path = rospy.ServiceProxy('/navigation/path_planning/a_star_search', GetPlan)
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest()
    req.start.pose.position.x = robot_x
    req.start.pose.position.y = robot_y
    req.goal.pose.position.x  = msg.pose.position.x
    req.goal.pose.position.y  = msg.pose.position.y
    path = clt_plan_path(req).plan
    print ("Following path with " + str(len(path.poses)) + " points...")
    path =[[p.pose.position.x, p.pose.position.y] for p in path.poses]
    follow_path(path)
    print ("Global goal point reached")

def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        return robot_x, robot_y, robot_a
    except:
        pass
    return None

def main():
    global pub_cmd_vel, loop, listener
    print ("PRACTICE 04 - " + NAME)
    rospy.init_node("practice04")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    loop = rospy.Rate(20)
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
    rospy.wait_for_service('/navigation/path_planning/a_star_search')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
