#!/usr/bin/env python
# coding=utf-8
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
# PRACTICE 1 - PATH PLANNING BY DIJKSTRA AND A-STAR
#
# Instructions:
# Write the code necessary to plan a path using two search algorithms:
# Dijkstra and A*
# MODIFY ONLY THE SECTIONS MARKED WITH THE COMMENT
#

import sys
import numpy as np
import heapq
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "ROJAS MOSQUEDA AXEL JAVIER"

def dijkstra(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    # grid_map tiene todos los puntos del mapa, es el arreglo del mapa
    # Write a Dijkstra algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Hint: Use a priority queue to implement the open list. 
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    g_values=np.full(np.shape(grid_map),sys.maxint)#Creo un arreglo para almacenar todas las g del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las g, en este caso me sirve para poder ir llenando las g de los primeros puntos
    p_nodes=np.full((np.shape(grid_map)[0],np.shape(grid_map)[1],2),-1)#Creo la lista de puntos anteriores para cada nodo con 2 dimensiones y la tercera con un valor de 2 ya que no lo necesitaremos, y los inicio con un valor de -1 para saber el limite
    in_open_list=np.full(np.shape(grid_map),False)#Estoy creando los verificadores de si esta en la lista cerrada o abierta y los llena de puros Falsos
    in_close_list=np.full(np.shape(grid_map),False)#Lista de puntos visitados
    
    open_list=[]#lista de puntos candidatos
    heapq.heappush(open_list,(0,[start_r,start_c]))#Agrego a la lista en la posicion 0 el punto inicial
    g_values[start_r,start_c]=0
    in_open_list[start_r,start_c]=True
    [r,c]=[start_r,start_c]#r,c sera el nodo actual, iniciando por la posicion del robot
    steps=0
    path=[]
    while len(open_list)>0 and [r,c]!=[goal_r,goal_c]:
        [r,c]=heapq.heappop(open_list)[1]#Estoy sacando el elemento con mayor prioridad y como me regresa la prioridad y el elemento entonces me interesa saolo sacar el elemento que es el 2 del arreglo
        in_close_list[r,c]=True#Decimos que ya visitamos el nodo que acabo de extraer 
        neighbors=[[r+1,c],[r-1,c],[r,c+1],[r,c-1]]#Sacamos los vecinos del punto actual
        for [nr,nc]in neighbors:#Para cada uno de los vecinos
            if grid_map[nr,nc]!=0 or in_close_list[nr,nc]:#Si el nodo vecino esta ocupado=100 o es desconocido=-1 o ya fue visitado se omite
                continue
            g=g_values[r,c]+1+cost_map[r,c]#Genero la g del punto anterios más 1, en este caso no es g=0 porque lo llene con un valor muy grande, pero no importa, ya que tiene el mismo efecto que empezar con g=0 y le agrego ademas una función de costos que es la distancia que hay de puntos a obstaculos ya que no quiero que el robot se acerque a obstaculos
            #f=g+(np.abs(r-nr)+np.abs(c-nc))
            if g<g_values[nr,nc]:#Si el costo del nodo vecino de r,c es menor al costo del nodo vecino nr, nc entonces cambio la g del nodo vecino, ya que con ello digo que al nodo vecino llegue por otra ruta mejor
                g_values[nr,nc]=g #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                p_nodes[nr,nc]=[r,c]#Aquí estoy agregando el predecesor del vecino si se encuentra que el costo es menor que el costo ya calculado por otra ruta tal vez
            if not in_open_list[nr,nc]: #Si el punto candidato no esta en la lista abierta entonces le cambio el valor, con esto podria agregar dicho punto a la lista abierta para buscar en ellos
                in_open_list[nr,nc]=True
                heapq.heappush(open_list,(g,[nr,nc]))#Agrego el punto candidato a la lista abierta si es que no estaba ya y le agrego la g para ordenar de menor g a mayor g e irlos sacando en ese orden
                
            steps+=1#Aumento el número de pasos que le tomo al algoritmo calcular la ruta y vuelvo a hacer lo mismo para cada punto de la lista abierta hasta terminar
       
    if[r,c]!=[goal_r,goal_c]:#Una vez checo para todos losn nodos, si el valor de r,c no es igual al objetivo es que no hay solucion
        print("Cannot calculate path by Dijsktra :c")
        return[]
    
    #Aqui r, c ya se quedo con los valores meta, por lo cual el punto anterior
    #Preguntar sobre p_nodes
    while [p_nodes[r,c][0],p_nodes[r,c][1]]!=[-1,-1]:#Sí si existe la solución entonces hasta encontrar el nodo [-1,-1] que fue con lo que se lleno y me dice que son todos los puntos
        path.insert(0,[r,c])#Voy agregando en la posicion cero cada una de las coordenadas que es el punto anterior
        [r,c]=p_nodes[r,c]
    print("The path has had "+str(steps)+" steps for been calculated")
    return path
        
    
    
def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # Write a A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Use Manhattan distance as heuristic function
    # Hint: Use a priority queue to implement the open list
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    #
    g_values=np.full(np.shape(grid_map),sys.maxint)#Creo un arreglo para almacenar todas las g del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las g, en este caso me sirve para poder ir llenando las g de los primeros puntos
    f_values=np.full(np.shape(grid_map),sys.maxint)#Creo un arreglo para almacenar todas las f del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las f, en este caso me sirve para poder ir llenando las f de los primeros puntos
    p_nodes=np.full((np.shape(grid_map)[0],np.shape(grid_map)[1],2),-1)#Creo la lista de puntos anteriores para cada nodo con 2 dimensiones y la tercera con un valor de 2 ya que no lo necesitaremos, y los inicio con un valor de -1 para saber el limite
    in_open_list=np.full(np.shape(grid_map),False)#Estoy creando los verificadores de si esta en la lista cerrada o abierta y los llena de puros Falsos
    in_close_list=np.full(np.shape(grid_map),False)#Lista de puntos visitados
    
    open_list=[]#lista de puntos candidatos
    heapq.heappush(open_list,(0,[start_r,start_c]))#Agrego a la lista en la posicion 0 el punto inicial
    g_values[start_r,start_c]=0
    f_values[start_r,start_c]=0
    in_open_list[start_r,start_c]=True
    [r,c]=[start_r,start_c]#r,c sera el nodo actual, iniciando por la posicion del robot
    steps=0
    path=[]
    while len(open_list)>0 and [r,c]!=[goal_r,goal_c]:
        [r,c]=heapq.heappop(open_list)[1]#Estoy sacando el elemento con mayor prioridad y como me regresa la prioridad y el elemento entonces me interesa saolo sacar el elemento que es el 2 del arreglo
        in_close_list[r,c]=True#Decimos que ya visitamos el nodo que acabo de extraer 
        neighbors=[[r+1,c],[r-1,c],[r,c+1],[r,c-1]]#Sacamos los vecinos del punto actual
        for [nr,nc]in neighbors:#Para cada uno de los vecinos
            if grid_map[nr,nc]!=0 or in_close_list[nr,nc]:#Si el nodo vecino esta ocupado=100 o es desconocido=-1 o ya fue visitado se omite
                continue
            g=g_values[r,c]+1+cost_map[r,c]#Genero la g del punto anterios más 1, en este caso no es g=0 porque lo llene con un valor muy grande, pero no importa, ya que tiene el mismo efecto que empezar con g=0 y le agrego ademas una función de costos que es la distancia que hay de puntos a obstaculos ya que no quiero que el robot se acerque a obstaculos
            f=g+(np.abs(goal_r-nr)+np.abs(goal_c-nc))#Calculo la f con la g y la distancia de manhatan
            if f<f_values[nr,nc]:#Si el costo del nodo vecino de r,c es menor al costo del nodo vecino nr, nc entonces cambio la g del nodo vecino, ya que con ello digo que al nodo vecino llegue por otra ruta mejor
                g_values[nr,nc]=g #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                f_values[nr,nc]=f #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                p_nodes[nr,nc]=[r,c]#Aquí estoy agregando el predecesor del vecino si se encuentra que el costo es menor que el costo ya calculado por otra ruta tal vez
            if not in_open_list[nr,nc]: #Si el punto candidato no esta en la lista abierta entonces le cambio el valor, con esto podria agregar dicho punto a la lista abierta para buscar en ellos
                in_open_list[nr,nc]=True
                heapq.heappush(open_list,(f,[nr,nc]))#Agrego el punto candidato a la lista abierta si es que no estaba ya y le agrego la g para ordenar de menor g a mayor g e irlos sacando en ese orden
                
            steps+=1#Aumento el número de pasos que le tomo al algoritmo calcular la ruta y vuelvo a hacer lo mismo para cada punto de la lista abierta hasta terminar
       
    if[r,c]!=[goal_r,goal_c]:#Una vez checo para todos losn nodos, si el valor de r,c no es igual al objetivo es que no hay solucion
        print("Cannot calculate path by Dijsktra :c")
        return[]
    
    #Aqui r, c ya se quedo con los valores meta, por lo cual el punto anterior
    #Preguntar sobre p_nodes
    while [p_nodes[r,c][0],p_nodes[r,c][1]]!=[-1,-1]:#Sí si existe la solución entonces hasta encontrar el nodo [-1,-1] que fue con lo que se lleno y me dice que son todos los puntos
        path.insert(0,[r,c])#Voy agregando en la posicion cero cada una de las coordenadas que es el punto anterior
        [r,c]=p_nodes[r,c]
    print("The path has had "+str(steps)+" steps for been calculated")
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
    inflated_map = np.asarray(inflated_map.data)
    inflated_map = np.reshape(inflated_map, (static_map.info.height, static_map.info.width))
    try:
        cost_map = clt_cost_map()
        cost_map = cost_map.map
    except:
        cost_map = static_map
        print("Cannot get cost map. Using static map instead")
    cost_map = np.asarray(cost_map.data)
    cost_map = np.reshape(cost_map, (static_map.info.height, static_map.info.width))
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
    print ("PRACTICE 01 - " + NAME)
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
    
