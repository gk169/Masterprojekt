#!/usr/bin/python3

# ## Create network
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')

import numpy as np
import rospy
import ros_numpy
import argparse

from model import LiDAR_Model
from laserscan import SemLaserScan
from pointcloud_handling import *

import yaml

from sensor_msgs.msg import PointCloud2

rospy.init_node('LiDAR_Net', anonymous=True)
pub_front = rospy.Publisher("/BugaSegm/pc_segm_front", PointCloud2, queue_size=1)
pub_top = rospy.Publisher("/BugaSegm/pc_segm_top", PointCloud2, queue_size=1)
pub_back = rospy.Publisher("/BugaSegm/pc_segm_back", PointCloud2, queue_size=1)

front_cloud = None
top_cloud = None
back_cloud = None

front_header = None
top_header = None
back_header = None

def PredictionToImage(Prediction):
    # Map masterproject classes to kitti classes 
    Prediction = ProjectToKitti_LUT[Prediction]
    # Map kitti classes to colors
    Image = KittiToColor_LUT[Prediction]
    Image = np.swapaxes(Image,0,1)
    Image = Image[...,[2,1,0]]
    return Image

# ## Predict BugaLog data
#current_pcd_path = args.pcd
def predict(): 
    global front_cloud
    global top_cloud
    global back_cloud
    global front_header
    global top_header
    global back_header
    
    rospy.loginfo("LiDAR_Net - predict")
    #PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)
        
    #current_sample = getSampleArrayFromPointCloud_pcd(PointCloud, current_pcd_path, args.sensor_height, args.layers)
    #print(current_sample.shape)
    #current_sample[:,:,2] = current_sample[:,:,2] -0.4#1.13 #+0.7 #- 1.13
        
    #current_sample = current_pcd
        
    if (front_cloud is None or top_cloud is None or back_cloud is None):
        return
        
    rospy.loginfo("LiDAR_Net - All topics received, start processing!")
        
    front_cloud_exp = np.expand_dims(front_cloud, axis=0)
    top_cloud_exp = np.expand_dims(top_cloud, axis=0)
    back_cloud_exp = np.expand_dims(back_cloud, axis=0)
    
    batch_cloud = np.concatenate((front_cloud_exp, top_cloud_exp, back_cloud_exp), axis=0)
    #print(batch_cloud.shape)
            
    Prediction = model.predict(batch_cloud)
        
    Prediction = np.argmax(Prediction,axis=3)
    #print(Prediction.shape)
    
    front_predict = Prediction[0,:,:]
    top_predict = Prediction[1,:,:]
    back_predict = Prediction[2,:,:]
    #print(front_predict.shape)
    
    #front_predict = front_predict.squeeze()
    #top_predict = top_predict.squeeze()
    #back_predict = back_predict.squeeze()
    #print(front_predict.shape)
    
    #plt.imsave('../data/images/front_predict_BugaLogImage.png', PredictionToImage(front_predict)) #TODO-perhaps remove
    #plt.imsave('../data/images/top_predict_BugaLogImage.png', PredictionToImage(top_predict)) #TODO-perhaps remove
    #plt.imsave('../data/images/back_predict_BugaLogImage.png', PredictionToImage(back_predict)) #TODO-perhaps remove
        
    # # Create result point cloud
    #intensity = Prediction
        
    #input_cloud = current_sample.squeeze()
    #cloud = input_cloud.copy()
        
    #print(cloud.shape)
    #print(Prediction.shape)
        
    # correct previous offset on z-axis (to kitti coords) to get original z-values
    front_cloud[:,:,2] = front_cloud[:,:,2] + 1.13
    top_cloud[:,:,2] = top_cloud[:,:,2] - 0.7
    back_cloud[:,:,2] = back_cloud[:,:,2] + 1.08
    if (args.layers == 'xyzi' or args.layers == 'xyzir'):
        front_cloud[:,:,3]=front_predict
        top_cloud[:,:,3]=top_predict
        back_cloud[:,:,3]=back_predict
    elif (args.layers == 'ir'):
        print("not implemented yet")
    elif (args.layers == 'xyz'):
        front_predict = np.expand_dims(front_predict, axis=2)
        top_predict = np.expand_dims(top_predict, axis=2)
        back_predict = np.expand_dims(back_predict, axis=2)
        front_cloud = np.append(front_cloud, front_predict, axis=2).astype(np.float32)
        top_cloud = np.append(top_cloud, top_predict, axis=2).astype(np.float32)
        back_cloud = np.append(back_cloud, back_predict, axis=2).astype(np.float32)
        
    #print(cloud.shape)
        
    #cloud[:,:,3]=Prediction
    #cloud=cloud[:,:,0:4]
    # überprüfen ob 0:4 oder 1:5
    #print(cloud.shape)
        
    #print(input_cloud[1,1,:])
    #print(cloud[1,1,:])
    
    front_cloud = np.reshape(front_cloud, (front_cloud.shape[0]*front_cloud.shape[1], 4))
    top_cloud = np.reshape(top_cloud, (top_cloud.shape[0]*top_cloud.shape[1], 4))
    back_cloud = np.reshape(back_cloud, (back_cloud.shape[0]*back_cloud.shape[1], 4))
    
    front_cloud = front_cloud[front_cloud[:,1]!= -1] # Remove image coordinates where no point was projected onto
    top_cloud = top_cloud[top_cloud[:,1]!= -1] # Remove image coordinates where no point was projected onto
    back_cloud = back_cloud[back_cloud[:,1]!= -1] # Remove image coordinates where no point was projected onto
    
    pub_front.publish(array_to_PointCloud2(front_cloud, front_header))
    pub_top.publish(array_to_PointCloud2(top_cloud, top_header))
    pub_back.publish(array_to_PointCloud2(back_cloud, back_header))
    
    #pcl_front = pcl.PointCloud_PointXYZI(front_cloud)
    #pcl_top = pcl.PointCloud_PointXYZI(top_cloud)
    #pcl_back = pcl.PointCloud_PointXYZI(back_cloud)
    
    #pcl.save(p, args.pcd.splitpcl_front = pcl.PointCloud_PointXYZI(front_cloud)
    #pcl_top = pcl.PointCloud_PointXYZI(top_cloud)
    #pcl_back = pcl.PointCloud_PointXYZI(back_cloud)('.pcd')[0] + "_Predicted.pcd") # TODO - publish statt safe
    
    front_cloud = None
    top_cloud = None
    back_cloud = None
    front_header = None
    top_header = None
    back_header = None

def veloFrontCallback(data):
    global front_cloud
    global front_header
    rospy.loginfo("LiDAR_Net - Received new PointCloud on /BugaSegm/synchronized/velodyne/front/velodyne_points")
    front_header = data.header
    
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],4))#,dtype="float32")
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)

    front_cloud = getSampleArrayFromPointCloud_pcd(PointCloud, points, -1.13, args.layers)
    
    predict()

def veloTopCallback(data):
    global top_cloud
    global top_header
    rospy.loginfo("LiDAR_Net - Received new PointCloud on /BugaSegm/synchronized/velodyne/top/velodyne_points")
    top_header = data.header
    
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],4))#,dtype="float32")
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)

    top_cloud = getSampleArrayFromPointCloud_pcd(PointCloud, points, 0.7, args.layers)
    
    predict()

def veloBackCallback(data):
    global back_cloud
    global back_header
    rospy.loginfo("LiDAR_Net - Received new PointCloud on /BugaSegm/synchronized/velodyne/back/velodyne_points")
    back_header = data.header
    
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],4))#,dtype="float32")
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)

    back_cloud = getSampleArrayFromPointCloud_pcd(PointCloud, points, -1.08, args.layers)
    
    predict()

def listener():
    rospy.Subscriber('/BugaSegm/synchronized/velodyne/front/velodyne_points', PointCloud2, veloFrontCallback)
    rospy.Subscriber('/BugaSegm/synchronized/velodyne/top/velodyne_points', PointCloud2, veloTopCallback)
    rospy.Subscriber('/BugaSegm/synchronized/velodyne/back/velodyne_points', PointCloud2, veloBackCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    ###############################
    # Import arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--layers', type=str, required=True) # 'xyz', 'xyzi', 'xyzir', 'ir'
    parser.add_argument('--weights', type=str, required=True)
    parser.add_argument('--yaml', type=str, required=True)
    #parser.add_argument('--pcd', type=str, required=True) #TODO - as topic subscribe
    #parser.add_argument('--sensor_height', type=float, required=True)
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
    
    # Load reduced model weights
    model.load_weights(args.weights)
    
    #start ros-listener
    listener()
