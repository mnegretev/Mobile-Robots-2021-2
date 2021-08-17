#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
# PRACTICE 1 - PATH PLANNING BY DIJKSTRA AND A-STAR
#
# Instructions:
# Write the code necessary to plan a path using two search algorithms:
# Dijkstra and A*
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import sys
import numpy
import heapq
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque
#from scipy.spatial import distance

NAME = "ESPINOBARROS_PERALTA"

def dijkstra(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write a Dijkstra algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Hint: Use a priority queue to implement the open list. 
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    #
        
   ########

    g_values = numpy.full (grid_map.shape,sys.maxint)
    parent_nodes = numpy.full ((grid_map.shape[0],grid_map.shape[1],2),-1)
    in_open_list = numpy.full(grid_map.shape,False) 
    in_closed_list = numpy.full(grid_map.shape,False)
    steps = 0
    
    open_list = []
    heapq.heappush(open_list, (0,[start_r,start_c]))
    g_values[start_r,start_c]=0 
    in_open_list[start_r,start_c]=True 
    [r,c]=[start_r,start_c]
    
    
    
    while len(open_list)>0 and [r,c]!=[goal_r,goal_c]:
        [r,c] = heapq.heappop(open_list)[1]
        in_closed_list[r,c]=True 
        neighbors=[[r+1,c],[r-1,c],[r,c+1],[r,c-1]]
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] != 0 or in_closed_list[nr,nc]:
                continue
            g = g_values[r,c] +1 + cost_map[nr][nc]
            if g < g_values[nr,nc]:
                g_values[nr,nc] = g 
                parent_nodes[nr,nc]=[r,c] 
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True  
                heapq.heappush(open_list,(g,[nr,nc]))
            steps+=1
        
        
    if [r,c]!=[goal_r,goal_c]: 
        print("Cannot calculate path by Dijsktra:'(")
        return [] 
    
    path=[]
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] !=[-1,-1]: 
        path.insert(0,[r,c])
        [r,c] = parent_nodes[r,c]
    print("Path calculated by Dijsktra afert"+str(steps)+" steps")
    return path

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write a A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Use Manhattan distance as heuristic function
    # Hint: Use a priority queue to implement the open list
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    #
    g_values =numpy.full (grid_map.shape,sys.maxint)
    f_values =numpy.full (grid_map.shape,sys.maxint)
    parent_nodes = numpy.full ((grid_map.shape[0],grid_map.shape[1],2),-1)
    in_open_list = numpy.full(grid_map.shape,False)
    in_closed_list = numpy.full(grid_map.shape,False)
    steps = 0
    
    open_list = []
    heapq.heappush(open_list, (0,[start_r,start_c]))
    g_values[start_r,start_c]=0 
    f_values[start_r,start_c]=0 
    in_open_list[start_r,start_c]=True 
    [r,c]=[start_r,start_c]
    
    while len(open_list)>0 and [r,c]!=[goal_r,goal_c]:
        [r,c] = heapq.heappop(open_list)[1]
        in_closed_list[r,c]=True
        neighbors=[[r+1,c],[r-1,c],[r,c+1],[r,c-1]]
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] != 0 or in_closed_list[nr,nc]:
                continue
            g = g_values[r,c] +1 + cost_map[nr][nc]
            f = abs(nr-goal_r)+abs(nc-goal_c) + g
            if g < g_values[nr,nc]:
                g_values[nr,nc] = g
                f_values[nr,nc] = f
                parent_nodes[nr,nc]=[r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True  
                heapq.heappush(open_list,(f,[nr,nc]))
            steps+=1
        
        
    if [r,c]!=[goal_r,goal_c]:
        print("Cannot calculate path by Dijsktra:'(")
        return [] 
    path=[]
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] !=[-1,-1]: 
        path.insert(0,[r,c])
        [r,c] = parent_nodes[r,c]
    print("Path calculated by A* after "+str(steps)+" steps")
    return path

def get_maps():
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    static_map   = clt_static_map()
    static_map   = static_map.map
    try:
        inflated_map = clt_inflated()
        inflated_map = inflated_map.map
    except:
        inflated_map = static_map
        print("Cannot get inflated map. Using static map instead")
    inflated_map = numpy.asarray(inflated_map.data)
    inflated_map = numpy.reshape(inflated_map, (static_map.info.height, static_map.info.width))
    try:
        cost_map = clt_cost_map()
        cost_map = cost_map.map
    except:
        cost_map = static_map
        print("Cannot get cost map. Using static map instead")
    cost_map = numpy.asarray(cost_map.data)
    cost_map = numpy.reshape(cost_map, (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map, cost_map]

def generic_callback(req, algorithm):
    [static_map, inflated_map, cost_map] = get_maps()
    
    [start_x, start_y] = [req.start.pose.position.x, req.start.pose.position.y]
    [goal_x,  goal_y ] = [req.goal.pose.position.x , req.goal.pose.position.y ]
    [zero_x,  zero_y ] = [static_map.info.origin.position.x,static_map.info.origin.position.y]
    [start_c, start_r] = [int((start_x - zero_x)/static_map.info.resolution), int((start_y - zero_y)/static_map.info.resolution)]
    [goal_c , goal_r ] = [int((goal_x  - zero_x)/static_map.info.resolution), int((goal_y  - zero_y)/static_map.info.resolution)]

    if algorithm == 'dijkstra':
        print("Calculating path by Dijkstra from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
        path = dijkstra(start_r, start_c, goal_r, goal_c, inflated_map, cost_map)
    else:
        print("Calculating path by A* from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
        path = a_star(start_r, start_c, goal_r, goal_c, inflated_map, cost_map)
    
    msg_path = Path()
    msg_path.header.frame_id = "map"
    for [r,c] in path:
        p = PoseStamped()
        p.pose.position.x = c*static_map.info.resolution + static_map.info.origin.position.x
        p.pose.position.y = r*static_map.info.resolution + static_map.info.origin.position.y
        msg_path.poses.append(p)
    pub_path = rospy.Publisher('/navigation/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    pub_path.publish(msg_path)
    return GetPlanResponse(msg_path)

def callback_dijkstra(req):
    return generic_callback(req, 'dijkstra')

def callback_a_star(req):
    return generic_callback(req, 'a_star')

def main():
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    rospy.Service('/navigation/path_planning/dijkstra_search', GetPlan, callback_dijkstra)
    rospy.Service('/navigation/path_planning/a_star_search'  , GetPlan, callback_a_star)
    rospy.wait_for_service('/static_map')
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
