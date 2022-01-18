#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from a_star import a_star_search
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import plot_path
import rospy
import numpy as np

origin=()
destination=()
#oc_grid = np.empty(1)

def start_callback(pose_msg):
    global origin
    x1 = pose_msg.pose.pose.position.x
    y1 = pose_msg.pose.pose.position.y

    res= 0.030054
    # converting real-time coordinates to occupancy grid indices 
    col1, row1= int((x1+50.01)/res) , int((y1+50.01)/res)
    origin = (row1, col1)


def goal_callback(goal_msg):
    global destination
    x2 = goal_msg.pose.position.x
    y2 = goal_msg.pose.position.y
    
    res= 0.030054
    # converting real-time coordinates to occupancy grid indices 
    col2, row2= int((x2+50.01)/res) , int((y2+50.01)/res)
    destination = (row2,col2)


def grid_callback(grid_msg):
    global oc_grid
    # create an object to store map data
    grid_oc= grid_msg.data

    #grid_oc is a 1D array. To convert it to 2D array we first convert
    #it to numpy array then reshape it
    oc_grid= np.array(grid_oc)
    oc_grid.reshape(3328,3328)

    

def path_search():
    global origin, destination
    print(origin)
    print(oc_grid.shape)
    print(destination)

    path= a_star_search(origin,destination,oc_grid)

    plot_path.plot(path)
    


if __name__ == '__main__':
    try:
        
        rospy.init_node('path_finder', anonymous=True)

        rospy.wait_for_message("/map",OccupancyGrid)
        grid_sub= rospy.Subscriber('/map', OccupancyGrid, grid_callback)
    
        # obtaining start position 
        rospy.wait_for_message("/ground_truth/state",Odometry)
        ori_sub = rospy.Subscriber('/ground_truth/state',Odometry, start_callback)
        
        # obtaining goal position
        rospy.wait_for_message("/move_base_simple/goal",PoseStamped, timeout= 50)
        destin_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)   
        
        
        path_search()
    
    except rospy.ROSInterruptException:
        pass
