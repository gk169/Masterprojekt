#!/usr/bin/env python
# coding: utf-8

# # LiDAR-Net

# ## Create network

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
import pcl
import math
import argparse

from laserscan import LaserScan, SemLaserScan
from model import LiDAR_Model

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

def getSampleArrayFromPointCloud_pcd (PointCloud, sample_path, sensor_height): #TODO - Sensorhöhe als input_param - Adjust relative to ground
    # Load point cloud from pcd
    cloud = pcl.load_XYZI(sample_path)
    cloud = cloud.to_array()
    # Get points and intensity (remission) from pcd point cloud
    points = cloud[:, 0:3]    # get xyz
    remissions = cloud[:, 3]/255 #normal remissions
    # Set points and remission to PointCloud
    PointCloud.set_points(points, remissions)
    xyz = PointCloud.proj_xyz.copy()
    
    #points[:,2] = points[:,2] - 1.13
    #camera 1,90
    #kitty 1,73
    #camera zu top +0,53
    #top 2,43
    #top zu front -1,83
    #front 0,6
    #front zu kitti 1,13
    #top zu kitti -0,7
    xyz[:,:,2] = xyz[:,:,2] + sensor_height
    
    # Get intensity and range from point cloud
    Intensity = PointCloud.proj_remission.copy()
    Range = PointCloud.proj_range.copy()
    
    # Expand dimensions to enable concatenate
    Intensity = np.expand_dims(Intensity,axis=2)
    Range = np.expand_dims(Range,axis=2)
    
    # Concatenate intensity and range according to given layers
    if (args.layers == 'xyzi'):
        xyzi = np.concatenate((xyz, Intensity),axis=2)
        xyzi = np.swapaxes(xyzi,0,1)
        return xyzi
    if (args.layers == 'xyzir'):
        xyzir = np.concatenate((xyz, Intensity, Range),axis=2)
        xyzir = np.swapaxes(xyzir,0,1)
        return xyzir
    if (args.layers == 'ir'):
        ir = np.concatenate((Intensity, Range),axis=2)
        ir = np.swapaxes(ir,0,1)
        return ir
    if (args.layers == 'xyz'):
        xyz = np.swapaxes(xyz,0,1)
        return xyz

    raise NameError('ERROR - Unknown layer type. Aborting!')

# Load reduced model weights
model.load_weights(args.weights)

def PredictionToImage(Prediction):
    # Map masterproject classes to kitti classes 
    Prediction = ProjectToKitti_LUT[Prediction]
    # Map kitti classes to colors
    Image = KittiToColor_LUT[Prediction]
    Image = np.swapaxes(Image,0,1)
    Image = Image[...,[2,1,0]]
    return Image

import matplotlib.pyplot as plt


# ## Predict BugaLog data
current_pcd_path = args.pcd
PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)

current_sample = getSampleArrayFromPointCloud_pcd(PointCloud, current_pcd_path, args.sensor_height)
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

#print(input_cloud.shape)
cloud = input_cloud.copy()
cloud[:,:,3]=Prediction
#cloud=cloud[:,:,0:4]
# überprüfen ob 0:4 oder 1:5
#print(cloud.shape)

#print(input_cloud[1,1,:])
#print(cloud[1,1,:])


cloud = np.reshape(cloud, (cloud.shape[0]*cloud.shape[1], 4))

cloud = cloud[cloud[:,1]!= -1] # Remove image coordinates where no point was projected onto

p = pcl.PointCloud_PointXYZI(cloud)

pcl.save(p, args.pcd.split('.pcd')[0] + "_Predicted.pcd") # TODO - publish statt safe

