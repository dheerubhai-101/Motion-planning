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
# global destination
# origin=()
# destination=()
# oc_grid = []

def start_callback(pose_msg):
    global origin
    x1 = pose_msg.pose.pose.position.x
    y1 = pose_msg.pose.pose.position.y

    res= 0.030054
    # converting real-time coordinates to occupancy grid indices 
    col1, row1= int((x1+50.01)/res) , int((y1+50.01)/res)
    origin = (row1, col1)


def goal_callback(goal_msg):
    
    x2 = goal_msg.pose.position.x
    y2 = goal_msg.pose.position.y
    print(x2)
    res= 0.030054
    # converting real-time coordinates to occupancy grid indices 
    col2, row2= int((x2+50.01)/res) , int((y2+50.01)/res)
    destination = (row2,col2)


def grid_callback(grid_msg):
    global oc_grid
    # create an object to store map data
    grid_oc= grid_msg.data
    oc_grid = grid_oc
    #grid_oc is a 1D array. To convert it to 2D array we first convert
    #it to numpy array then reshape it
    oc_grid= np.array(oc_grid)
    oc_grid.reshape(3328,3328)

    

def path_search(origin,destination,oc_grid):
    print(origin)
    print(oc_grid.shape)
    print(destination)

    path= a_star_search(origin,destination,oc_grid)

    plot_path.plot(path)
    


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

        
        
        
        path_search(origin, destination, oc_grid)
    
    except rospy.ROSInterruptException:
        pass
