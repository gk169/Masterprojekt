#!/usr/bin/env python
# coding: utf-8

# # LiDAR-Net

# ## Create network

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import argparse

from model import LiDAR_Model
from laserscan import LaserScan, SemLaserScan
from pointcloud_handling import *

import yaml

###############################
# Import arguments
parser = argparse.ArgumentParser()
parser.add_argument('--layers', type=str, required=True) # 'xyz', 'xyzi', 'xyzir', 'ir'
parser.add_argument('--weights', type=str, required=True)
parser.add_argument('--yaml', type=str, required=True)
parser.add_argument('--pcd', type=str, required=True) #TODO - as topic subscribe
parser.add_argument('--sensor_height', type=float, required=True)
args, unknown = parser.parse_known_args()
###############################

def PredictionToImage(Prediction):
    # Map masterproject classes to kitti classes 
    Prediction = ProjectToKitti_LUT[Prediction]
    # Map kitti classes to colors
    Image = KittiToColor_LUT[Prediction]
    Image = np.swapaxes(Image,0,1)
    Image = Image[...,[2,1,0]]
    return Image

# Generate color map and lookup tables
# Load configuration file
CFG = yaml.safe_load(open(args.yaml,'r'))

# Read kitti classes to color dictionary from configuration
KittiToColorDict = CFG['color_map']

# Read kitti to master project classes dictionary from configuration
KittiToProjectDict = CFG['learning_map']

# Read master project to kitti dictionary from configuration
ProjectToKittiDict = CFG['learning_map_inv']

# Create lookup table for kitti classes to color
maxkeyColor = max(KittiToColorDict.keys()) + 100 # +100 hack making lut bigger in case there are unknown labels
KittiToColor_LUT = np.zeros((maxkeyColor, 3), dtype=np.uint8)
KittiToColor_LUT[list(KittiToColorDict.keys())] = list(KittiToColorDict.values())

# Create lookup table for kitti classes to master project classes
maxkey = max(KittiToProjectDict.keys()) + 100 # +100 hack making lut bigger in case there are unknown labels 
maxvalue = max(KittiToProjectDict.values())
KittiToProject_LUT = np.zeros((maxkey), dtype=np.int32)
KittiToProject_LUT[list(KittiToProjectDict.keys())] = list(KittiToProjectDict.values())

# Create lookup table for master project classes to kitti classes
maxkeyInv = max(ProjectToKittiDict.keys()) + 100 # +100 hack making lut bigger in case there are unknown labels
ProjectToKitti_LUT = np.zeros((maxkeyInv), dtype=np.int32)
ProjectToKitti_LUT[list(ProjectToKittiDict.keys())] = list(ProjectToKittiDict.values())

model = LiDAR_Model(len(args.layers), CFG['num_classes'])

# Load reduced model weights
model.load_weights(args.weights)

# ## Predict BugaLog data
current_pcd_path = args.pcd
PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)

current_sample = getSampleArrayFromPointCloud_pcd(PointCloud, current_pcd_path, args.sensor_height, args.layers)
#print(current_sample.shape)
#current_sample[:,:,2] = current_sample[:,:,2] -0.4#1.13 #+0.7 #- 1.13

#current_sample = current_pcd
current_sample = np.expand_dims(current_sample, axis=0)

#current_sample = current_sample[0::,0::2,0::4]
#current_sample[np.where(current_sample[:,:,:,0] > 0.4)] = [-1, -1]

Prediction = model.predict(current_sample)

Prediction = np.argmax(Prediction,axis=3)

Prediction = Prediction.squeeze()

BugaLogImage = PredictionToImage(Prediction)

plt.imsave('../data/images/BugaLogImage.png', BugaLogImage) #TODO-perhaps remove

# # Create result point cloud
intensity = Prediction

input_cloud = current_sample.squeeze()
cloud = input_cloud.copy()

#print(cloud.shape)
#print(Prediction.shape)
if (args.layers == 'xyzi'):
    cloud[:,:,3]=Prediction
elif (args.layers == 'xyzir'):
    cloud[:,:,3]=Prediction
elif (args.layers == 'ir'):
    print("not implemented yet")
elif (args.layers == 'xyz'):
    Prediction = np.expand_dims(Prediction, axis=2)
    cloud = np.append(cloud, Prediction, axis=2).astype(np.float32)

#print(cloud.shape)

#cloud[:,:,3]=Prediction
#cloud=cloud[:,:,0:4]
# überprüfen ob 0:4 oder 1:5
#print(cloud.shape)

#print(input_cloud[1,1,:])
#print(cloud[1,1,:])


cloud = np.reshape(cloud, (cloud.shape[0]*cloud.shape[1], 4))

cloud = cloud[cloud[:,1]!= -1] # Remove image coordinates where no point was projected onto

p = pcl.PointCloud_PointXYZI(cloud)

pcl.save(p, args.pcd.split('.pcd')[0] + "_Predicted.pcd") # TODO - publish statt safe

