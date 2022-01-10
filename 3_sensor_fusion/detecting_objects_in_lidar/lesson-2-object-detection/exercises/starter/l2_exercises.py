# ---------------------------------------------------------------------
# Exercises from lesson 2 (object detection)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

from PIL import Image
import io
import sys
import os
import cv2
import open3d as o3d
import math
import numpy as np
import zlib

import matplotlib    
import matplotlib.pyplot as plt

# Exercise C2-4-6 : Plotting the precision-recall curve

"""
NOTE: it would then be possible to compute the average precision
"""

def plot_precision_recall():
    # Please note: this function assumes that you have pre-computed the precions/recall value pairs from the test sequence
    #              by subsequently setting the variable configs.conf_thresh to the values 0.1 ... 0.9 and noted down the results.
    
    # Please create a 2d scatter plot of all precision/recall pairs 
    # iou                        0.5,                      0.1
    P = [0.97, 0.94, 0.93, 0.92, 0.915, 0.91, 0.89, 0.87, 0.82]
    R = [0.738, 0.738, 0.743, 0.746, 0.746, 0.747, 0.748, 0.752, 0.754]
    
    plt.scatter(R, P)

    # needed to show plt when using python command line
    plt.show()
    
    
    
# Exercise C2-3-4 : Compute precision and recall

""" 
NOTE:

we do not have information on the location of these errors: Did they occur within the path of driving? How long did phantom objects pop up? For how many frames did actual vehicles disappear? In practice, the engineering team would need to investigate each case to analyze the circumstances of these errors. Also, as problems such as missed detections and short-lived phantom objects can be removed or at least mitigated by the tracking stage (see lessons after mid-term), it makes sense to not look at frame-to-frame detections but rather at the object tracks over time.

Based on the precision result, we can say that the probability for a detection being an actual vehicle is at ~92%. The recall result provides us with the information that the probability for a vehicle being detected is at ~75%. Note that both values only hold for a confidence threshold of 0.5.
"""

def compute_precision_recall(det_performance_all, conf_thresh=0.5):

    if len(det_performance_all)==0 :
        print("no detections for conf_thresh = " + str(conf_thresh))
        return
    
    # extract the total number of positives, true positives, false negatives and false positives
    # format of det_performance_all is [ious, center_devs, pos_negs]

    
    # Udacity solution
#     pos_negs = []
#     for item in det_performance_all:
#         pos_negs.append(item[2])
#     pos_negs_arr = np.asarray(pos_negs)        

    # NOTE: I think here, positives means object actually exist,
    # positives = true_positives + false_negatives
    
#     positives = sum(pos_negs_arr[:,0])
#     true_positives = sum(pos_negs_arr[:,1])
#     false_negatives = sum(pos_negs_arr[:,2])
#     false_positives = sum(pos_negs_arr[:,3])
    
    
    # my solution
    positives = 0
    true_positives = 0
    false_negatives = 0
    false_positives = 0
    for item in det_performance_all:
        print('item', item)
        pos_negs = item[2]
        
        positives += pos_negs[0]
        true_positives += pos_negs[1]
        false_negatives += pos_negs[2]
        false_positives += pos_negs[3]
    
    
    
    print("Overall TP = " + str(true_positives) + ", FP = " + str(false_positives) + ", FN = " + str(false_negatives))                    
    
    
    # compute precision
    precision = true_positives / (true_positives + false_positives)    
    
    # compute recall 
    recall = true_positives / (true_positives + false_negatives)

    print("precision = " + str(precision) + ", recall = " + str(recall) + ", conf_thres = " + str(conf_thresh) + "\n")    
    


    
    
# Exercise C2-3-2 : Transform metric point coordinates to BEV space
def pcl_to_bev(lidar_pcl, configs, vis=True):

    # compute bev-map discretization by dividing x-range by the bev-image height
    # NOTE: lidar x axis is for bev height axis, lidar y axis is for bev width axis
    bev_discret = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height

    # create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates   
    lidar_pcl_cpy = np.copy(lidar_pcl)
    lidar_pcl_cpy[:, 0] = np.int_(np.floor(lidar_pcl_cpy[:, 0] / bev_discret))

    # transform all metrix y-coordinates as well but center the foward-facing x-axis on the middle of the image
    lidar_pcl_cpy[:, 1] = np.int_( np.floor(lidar_pcl_cpy[:, 1] / bev_discret) + (configs.bev_width + 1) / 2 )

    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl_cpy[:, 2] = lidar_pcl_cpy[:, 2] - configs.lim_z[0]  

    

    
    # re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then by decreasing height
    idx_height = np.lexsort((-lidar_pcl_cpy[:, 2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))
    lidar_pcl_hei = lidar_pcl_cpy[idx_height]

    # extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    _, idx_height_unique = np.unique(lidar_pcl_hei[:, 0:2], axis=0, return_index=True)
    lidar_pcl_hei = lidar_pcl_hei[idx_height_unique]

    # assign the height value of each unique entry in lidar_top_pcl to the height map and 
    # make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    height_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))

    # NOTE: lidar x axis is for bev height axis, lidar y axis is for bev width axis
    height_map[np.int_(lidar_pcl_hei[:, 0]), np.int_(lidar_pcl_hei[:, 1])] = lidar_pcl_hei[:, 2] / float(np.abs(configs.lim_z[1] - configs.lim_z[0]))
    
    # visualize height map
    if vis:
        img_height = (height_map * 255).astype(np.uint8)
        cv2.imshow('img_height', img_height)
        cv2.waitKey(0)


    
    
    # sort points such that in case of identical BEV grid coordinates, the points in each grid cell are arranged based on their intensity
    lidar_pcl_cpy[lidar_pcl_cpy[:, 3] > 1.0, 3] = 1.0
    idx_intensity = np.lexsort((-lidar_pcl_cpy[:, 3], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))
    lidar_pcl_cpy = lidar_pcl_cpy[idx_intensity]

    # only keep one point per grid cell
    _, indices = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True)
    lidar_pcl_int = lidar_pcl_cpy[indices]

    # create the intensity map
    intensity_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    intensity_map[np.int_(lidar_pcl_int[:, 0]), np.int_(lidar_pcl_int[:, 1])] = lidar_pcl_int[:, 3] / (np.amax(lidar_pcl_int[:, 3])-np.amin(lidar_pcl_int[:, 3]))

    # visualize intensity map
    if vis:
#         img_intensity = intensity_map * 256
        img_intensity = (intensity_map * 255).astype(np.uint8)

#         while (1):
#             cv2.imshow('img_intensity', img_intensity)
#             if cv2.waitKey(10) & 0xFF == 27:
#                 break    
#         cv2.destroyAllWindows()


        cv2.imshow('img_intensity', img_intensity)
        cv2.waitKey(0)

