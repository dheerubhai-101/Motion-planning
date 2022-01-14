#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 14:19:17 2022

@author: dheeraj
"""
import rospy
from a_star import a_star_search
import plot_path

def path_search():

    

    origin= (370,370)
    destination= (639,269)

    path= a_star_search(origin,destination)


    plot_path.plot(path)

if __name__ == '__main__':
    try:
        path_search()
       

    except rospy.ROSInterruptException:
        pass

        