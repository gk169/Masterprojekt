#!/usr/bin/env python
# coding: utf-8

# # LiDAR-Net

# ## Create network

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.layers import Input, Conv2D, AveragePooling2D, Conv2DTranspose
from tensorflow.keras.layers import BatchNormalization, LeakyReLU, Dropout
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras import backend as K
import os
import pcl
import math
import argparse

from laserscan import LaserScan, SemLaserScan
import yaml

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

#TODO - auslagern des Netzes in eigenes .py

###############################
# Import arguments
parser = argparse.ArgumentParser()
parser.add_argument('--layers', type=str, required=True) # 'xyz', 'xyzi', 'xyzir', 'ir'
parser.add_argument('--weights', type=str, required=True)
parser.add_argument('--yaml', type=str, required=True)
parser.add_argument('--pcd', type=str, required=True) #TODO - as topic subscribe
parser.add_argument('--sensor_height', type=float, required=True)
args, unknown = parser.parse_known_args()

#weights_path = 
#input_layers = ... # 'xyz', 'xyzi', 'xyzir', 'ir'
#yaml_path = ...
#cloud_path = ...
###############################

class ResContextBlock(tf.keras.Model):
    def __init__(self, out_filters):
        super(ResContextBlock, self).__init__()
        self.conv1 = Conv2D(out_filters, kernel_size=(1, 1), strides=1)
        self.act1 = LeakyReLU()

        self.conv2 = Conv2D(out_filters, kernel_size=(3,3), padding='same')
        self.act2 = LeakyReLU()
        self.bn1 = BatchNormalization()

        self.conv3 = Conv2D(out_filters, kernel_size=(3,3), dilation_rate=2, padding='same')
        self.act3 = LeakyReLU()
        self.bn2 = BatchNormalization()


    def call(self, x):

        shortcut = self.conv1(x)
        shortcut = self.act1(shortcut)

        resA = self.conv2(shortcut)
        resA = self.act2(resA)
        resA1 = self.bn1(resA)

        resA = self.conv3(resA1)
        resA = self.act3(resA)
        resA2 = self.bn2(resA)

        output = shortcut + resA2
        return output

class ResBlock(tf.keras.Model):
    def __init__(self, out_filters, dropout_rate, kernel_size=(3, 3), stride=1,
                 pooling=True, drop_out=True):
        super(ResBlock, self).__init__()
        self.pooling = pooling
        self.drop_out = drop_out
        self.conv1 = Conv2D(out_filters, kernel_size=(1, 1), strides=stride)
        self.act1 = LeakyReLU()

        self.conv2 = Conv2D(out_filters, kernel_size=(3, 3), padding='same')
        self.act2 = LeakyReLU()
        self.bn1 = BatchNormalization()

        self.conv3 = Conv2D(out_filters, kernel_size=(3,3),dilation_rate=2, padding='same')
        self.act3 = LeakyReLU()
        self.bn2 = BatchNormalization()

        self.conv4 = Conv2D(out_filters, kernel_size=(2, 2), dilation_rate=2, padding='same')
        self.act4 = LeakyReLU()
        self.bn3 = BatchNormalization()

        self.conv5 = Conv2D(out_filters, kernel_size=(1, 1))
        self.act5 = LeakyReLU()
        self.bn4 = BatchNormalization()

        if pooling:
            self.dropout = Dropout(dropout_rate)
            self.pool = AveragePooling2D(pool_size=kernel_size, strides=2, padding='same')
        else:
            self.dropout = Dropout(dropout_rate)

    def call(self, x):
        shortcut = self.conv1(x)
        shortcut = self.act1(shortcut)

        resA = self.conv2(x)
        resA = self.act2(resA)
        resA1 = self.bn1(resA)

        resA = self.conv3(resA1)
        resA = self.act3(resA)
        resA2 = self.bn2(resA)

        resA = self.conv4(resA2)
        resA = self.act4(resA)
        resA3 = self.bn3(resA)

        concat = tf.concat((resA1,resA2,resA3),axis=-1)
        resA = self.conv5(concat)
        resA = self.act5(resA)
        resA = self.bn4(resA)
        resA = shortcut + resA


        if self.pooling:
            if self.drop_out:
                resB = self.dropout(resA)
            else:
                resB = resA
            resB = self.pool(resB)
            return resB, resA
        else:
            if self.drop_out:
                resB = self.dropout(resA)
            else:
                resB = resA
            return resB

class UpBlock(tf.keras.Model):
    def __init__(self, out_filters, dropout_rate, drop_out=True):
        super(UpBlock, self).__init__()
        self.drop_out = drop_out
        self.out_filters = out_filters

        self.dropout1 = Dropout(dropout_rate)
        self.dropout2 = Dropout(dropout_rate)

        self.conv1 = Conv2D(out_filters, kernel_size=(3,3), padding='same')
        self.act1 = LeakyReLU()
        self.bn1 = BatchNormalization()

        self.conv2 = Conv2D(out_filters, kernel_size=(3,3),dilation_rate=2, padding='same')
        self.act2 = LeakyReLU()
        self.bn2 = BatchNormalization()

        self.conv3 = Conv2D(out_filters, kernel_size=(2,2), dilation_rate=2,padding='same')
        self.act3 = LeakyReLU()
        self.bn3 = BatchNormalization()


        self.conv4 = Conv2D(out_filters,kernel_size=(1,1))
        self.act4 = LeakyReLU()
        self.bn4 = BatchNormalization()
        self.dropout3 = Dropout(dropout_rate)

    def call(self, x, skip):
        upA = tf.nn.depth_to_space(x,2)
        if self.drop_out:
            upA = self.dropout1(upA)

        upB = tf.concat((upA,skip),axis=-1)
        if self.drop_out:
            upB = self.dropout2(upB)

        upE = self.conv1(upB)
        upE = self.act1(upE)
        upE1 = self.bn1(upE)

        upE = self.conv2(upE1)
        upE = self.act2(upE)
        upE2 = self.bn2(upE)

        upE = self.conv3(upE2)
        upE = self.act3(upE)
        upE3 = self.bn3(upE)

        concat = tf.concat((upE1,upE2,upE3),axis=-1)
        upE = self.conv4(concat)
        upE = self.act4(upE)
        upE = self.bn4(upE)
        if self.drop_out:
            upE = self.dropout3(upE)

        return upE

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

INPUT_SHAPE = [None,None,len(args.layers)]
# Input shape kitti dataset
#INPUT_SHAPE = [2048,64,5]
# input shape of bugalog data
#INPUT_SHAPE = [1024,16,4]
# min input shape
#INPUT_SHAPE = [16,16,5]

K.clear_session()

input_layer = Input(shape=INPUT_SHAPE)
x = ResContextBlock(32)(input_layer)
x = ResContextBlock(32)(x)
x = ResContextBlock(32)(x)
x,resBlock1 = ResBlock(64, 0.2, pooling=True, drop_out=False)(x)

x,resBlock2 = ResBlock(128, 0.2, pooling=True)(x)
x,resBlock3 = ResBlock(128, 0.2, pooling=True)(x)
x,resBlock4 = ResBlock(256, 0.2, pooling=True)(x)

x = ResBlock(256, 0.2, pooling=False)(x)

x = UpBlock(128, 0.2)(x,resBlock4)
x = UpBlock(128, 0.2)(x,resBlock3)
x = UpBlock(64, 0.2)(x,resBlock2)
x = UpBlock(32, 0.2, drop_out=False)(x,resBlock1)

logits = Conv2D(CFG['num_classes'], kernel_size=(1, 1), activation="softmax")(x)

model = Model(inputs=input_layer, outputs=logits)
#model.summary()


# ## Prepare data
BatchSize = 4 # für 64er pc => BatchSize=4 #für 16er pc & 5,xMill. Param => 32 
Epochs = 10
LearningRate = 1e-5

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

Optimizer = Adam(learning_rate = LearningRate)
# Configure model for training
model.compile(
loss='categorical_crossentropy',
optimizer=Optimizer,
metrics=['accuracy']
)

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
print(np.max(current_sample[:,:,2]))
print(np.min(current_sample[:,:,2]))

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

print(input_cloud.shape)
cloud = input_cloud.copy()
cloud[:,:,3]=Prediction
#cloud=cloud[:,:,0:4]
# überprüfen ob 0:4 oder 1:5
print(cloud.shape)

#print(input_cloud[1,1,:])
#print(cloud[1,1,:])


cloud = np.reshape(cloud, (cloud.shape[0]*cloud.shape[1], 4))

cloud = cloud[cloud[:,1]!= -1] # Remove image coordinates where no point was projected onto

p = pcl.PointCloud_PointXYZI(cloud)

pcl.save(p, args.pcd.split('.pcd')[0] + "_Predicted.pcd") # TODO - publish statt safe

