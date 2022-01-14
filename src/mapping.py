# -*- coding: utf-8 -*-
"""
Created on Thu Jan  6 17:48:30 2022

@author: karthik
"""

# import opencv
import cv2
import numpy as np

def threshold(image):
    g_image=(image)
    p= np.shape(image)[0]
    q= np.shape(image)[1]
    for i in range(0,p):
        for j in range(0,q):
          if g_image[i,j]>220:
               g_image[i,j]=255
          elif g_image[i][j] ==205:
               g_image[i,j]=0
               
    return g_image

   
def map_threshold(image):    
    # Load the input image

    
    # Use the cvtColor() function to grayscale the image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    #cv2.imshow('Grayscale', gray_image)
    cv2.waitKey(0)
    
    # Window shown waits for any key pressing event
    cv2.destroyAllWindows()
    
    img=threshold(gray_image)
    #cv2.imshow('map',img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return img
