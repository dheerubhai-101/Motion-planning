#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import rospy
import numpy as np
from nav_msgs.msg import Path
from math import pow,atan2,sqrt,asin
import priority_dict
from math import dist,sqrt
from std_msgs.msg import Float64
import math


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def get_nearest_pixels(node,image):
    pix_list=[]
    y,x = node
    for i in range(-1,2):
        for j in range(-1,2):
            t1,t2= y+i,x+j
            pixel=image[t1,t2]
            if t1==y and t2==x:
                continue
            elif pixel==0:
                p= [y,x]
                q= [t1,t2]
                length= dist(p, q)
                
                pix_list.append((t1,t2,'%.2f'%length))
            elif pixel==100:
                length = 100000
            elif pixel== -1:
                continue
    return pix_list

# This function follows the predecessor
# backpointers and generates the equivalent path from the
# origin as a list of vertex keys.
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path


def distance_heuristic(current_node,goal_node):
    y1,x1 = current_node
    y2,x2 = goal_node
    d= sqrt((y2-y1)**2 + (x2-x1)**2)
    d= float('%.3f'%d)
    return d

def a_star_search(origin_key, goal_key, image):
    # The priority queue of open vertices we've reached.
    # Keys are the vertex keys, vals are the accumulated
    # distances plus the heuristic estimates of the distance
    # to go.
    open_queue = priority_dict.priority_dict()
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    predecessors[origin_key]= 0
    
    # The dictionary that stores the best cost to reach each
    # vertex found so far.
    costs = {}
    
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key, goal_key)

    
    goal_found = False
    while (open_queue):
        u=open_queue.pop_smallest()
        #u=(U[0],U[1])
        #u_val=U[1]
        uCost= costs[u]
        
        if u==goal_key: 
            goal_found= True
            break
            
        
        for edge in get_nearest_pixels(u,image):
            v=(edge[0], edge[1])
            uvCost = float(edge[2])
            h_v= distance_heuristic(v, goal_key)
            
            if v in closed_dict:
                continue
            
            if v in open_queue:
                if uCost+uvCost+h_v< open_queue[v]:
                    open_queue[v]= uCost+uvCost+h_v
                    costs[v]= uCost+uvCost
                    predecessors[v]=u
            
            else:
                open_queue[v]= uCost+uvCost+h_v
                costs[v]= uCost+uvCost
                predecessors[v]=u
                

        
        closed_dict[u]=uCost
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)   


def callback(pose_msg):
    global x,y,xq,yq,zq,wq
    x= pose_msg.pose.pose.position.x
    y= pose_msg.pose.pose.position.x

    # u= pose_msg.twist.twist.linear.x
    # v= pose_msg.twist.twist.linear.y
    # c= sqrt(u**2+v**2)

    # w= pose_msg.twist.twist.angular.z
    xq = pose_msg.pose.pose.orientation.x
    yq = pose_msg.pose.pose.orientation.y
    zq = pose_msg.pose.pose.orientation.z
    wq = pose_msg.pose.pose.orientation.w    

def path_search(origin,destination,oc_grid,a,b):
    
    route= a_star_search(origin,destination,oc_grid)
    path= Path()
    path.header.frame_id = "map"
    res= 0.030054
    track= list()

    for i in route:
        row , col = i
        y = res*row - 50.01
        x = res*col - 50.01
        point = x , y
        #add point to route
        track.append(point)
    
    rate= rospy.Rate(5)    

    track.append((a,b))
    return track    

                  #To find difference in desired and current orientation
            #If positive, then robot has to turn counter-clockwise
            #If negative, then robot has to turn clockwise
            #If zero, then robot moves forward
            # F = theta_star - yaw_z
            # F = float('%.2f'%F)   
        # if pressed_key=="s":
        #     pub[0].publish(-10)
        #     pub[1].publish(10)
        #     pub[2].publish(-10)
        #     pub[3].publish(10)

        #If robot has to turn left    
        # elif F>0:
        #     pub[0].publish(W)
        #     pub[1].publish(W)
        #     pub[2].publish(W)
        #     pub[3].publish(W)
            
            
        # elif F<0:
        #     pub[0].publish(W)
        #     pub[1].publish(W)
        #     pub[2].publish(W)
        #     pub[3].publish(W)


def joint_name(number):
    name= '/joint'+ str(number)+'_vel_controller/command'
    return name

if __name__ == '__main__':
    try:
        
        rospy.init_node('path_finder', anonymous=True)

        # obtaining grid
        grid=rospy.wait_for_message("/map",OccupancyGrid)
       # create an object to store map data
        grid_oc= grid.data

        #grid_oc is a 1D array. To convert it to 2D array we first convert
        #it to numpy array then reshape it
        oc_grid= np.array(grid_oc)
        oc_grid= np.reshape(oc_grid,(3328,3328))

        # obtaining start position 
        start=rospy.wait_for_message("/ground_truth/state",Odometry)
        
        pos = rospy.Subscriber("/ground_truth/state",Odometry, callback)

        x1 = start.pose.pose.position.x
        y1 = start.pose.pose.position.y

        res= 0.030054
        # converting real-time coordinates to occupancy grid indices 
        col1, row1= int((x1+50.01)/res) , int((y1+50.01)/res)
        origin = (row1, col1)


        
        # obtaining goal position
        goal=rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
    
        x2 = goal.pose.position.x
        y2 = goal.pose.position.y

        res= 0.030054
        # converting real-time coordinates to occupancy grid indices 
        col2, row2= int((x2+50.01)/res) , int((y2+50.01)/res)
        destination = (row2,col2)

        rate= rospy.Rate(1)

        joints=[]
        pub=[]
        total_joints=4

        for i in range(total_joints):
            joints.append(joint_name(i+1))
            pub.append(rospy.Publisher(joints[i], Float64 , queue_size=10))
        
    


        track= path_search(origin, destination, oc_grid,x2,y2) #A* search path as a list of coordinates


        R= 0.08
        L= 0.41
        w_int= 0
        old_e = 0
        E = 0
        w=0
        for point in track:
            roll_x, pitch_y, yaw_z = euler_from_quaternion(xq, yq, zq, wq)
            des_x , des_y = point
            K_v=5
            K_d=10
            K_i= 3
            error= sqrt(pow(des_x-x,2)+pow(des_y-y,2)) #Proportional term
            e_dot= error - old_e #Differential term
            E = E + error #Integral term
            V= K_v*error + K_d*e_dot + K_i*E
            old_e = error
            
            theta_star= atan2(des_y-y,des_x-x)
            k_t= 3
            k_d= 5
            k_i= 2

  
            theta= yaw_z
            w_star= theta_star-theta
            w_dot= w_star-w
            w_int= w_int + w_star 
            W=k_t*w_star+ k_d*w_dot + k_i*w_int
            
            w= w_star

            Vl = V/R - (W*L)/(2*R)
            Vr = V/R + (W*L)/(2*R)
        
            pub[0].publish(Vl)
            pub[1].publish(-Vr)
            pub[2].publish(Vl)
            pub[3].publish(-Vr)
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
