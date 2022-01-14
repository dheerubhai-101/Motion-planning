#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 15:01:21 2022

@author: dheeraj
"""

import rospy
from nav_msgs.msg import Path
#import numpy as np

# changing colour of pixels in the path

def plot(path):
    #Initializing node
    #path= Path()
    #path= poses
    rospy.init_node('Path',anonymous=True)
    
    path_topic= 'TrajectoryPlannerROS/global_plan'
    
    #creating a publisher for path
    path_pub= rospy.Publisher(path_topic, Path, queue_size=10)

    #publishing the topic to rviz
    path_pub.publish(path)

    pass