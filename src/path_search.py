#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 14:19:17 2022

@author: dheeraj
"""
import rospy
from a_star import a_star_search
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import plot_path
import rospy
import numpy as np




def back_grid(msg):
    global oc_grid
    grid_oc= msg.data
    oc_grid= np.array(grid_oc)
    oc_grid.reshape(850,700)

    #return grid_oc

def back_nation(target):
    global goal
    dx = target.pose.position.x
    dy = target.pose.position.y
    dz = target.pose.position.z
    goal = (dx,dy)
    # goal = Point()
    # goal = target.pose.position
    # return goal

def back_ground(loc):
    global pos
    ox = loc.pose.pose.position.x
    oy = loc.pose.pose.position.y
    oz = loc.pose.pose.position.z    
    pos = (ox,oy)
    # pos = Point()
    # # pos.x = loc.pose.pose.x
    # # pos.y = loc.pose.pose.y
    # # pos.z = loc.pose.pose.z
    # pos = loc.pose.pose.position
    # return pos

def path_search():
    while not rospy.is_shutdown():
        rospy.init_node('path_finder', anonymous=True)
        # obtaining start position 
        rospy.wait_for_message("/ground_truth/state",Odometry)
        ori_sub = rospy.Subscriber('/ground_truth/state',Odometry, back_ground)
        
        origin = pos
        # obtaining goal position
        rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        destin_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, back_nation )
        destination= goal    
        # create an object to store map data
        rospy.wait_for_message("/map",OccupancyGrid)
        grid_sub= rospy.Subscriber('/map', OccupancyGrid, back_grid)
               
        
        
        #grid_oc is a 1D array. To convert it to 2D array we first convert
        #it to numpy array then reshape it
        



        path= a_star_search(origin,destination,oc_grid)

        plot_path.plot(path)
        rospy.spin()

if __name__ == '__main__':
    try:
         
        
        path_search()
    
    except rospy.ROSInterruptException:
        pass

        