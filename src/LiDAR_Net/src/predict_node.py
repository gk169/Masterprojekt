#!/usr/bin/python3

import numpy as np
import rospy
import ros_numpy
import argparse
import threading

mutex = threading.Lock()

from model import LiDAR_Model
from laserscan import SemLaserScan
from pointcloud_handling import array_to_PointCloud2, getSampleArrayFromPointCloud_pcd
from scipy.spatial.transform import Rotation as R

import yaml

from sensor_msgs.msg import PointCloud2

rospy.init_node('LiDAR_Net', anonymous=True)
#pub_front = rospy.Publisher("/BugaSegm/pc_segm_front", PointCloud2, queue_size=1)
pub_top = rospy.Publisher("/BugaSegm/pc_segm_top", PointCloud2, queue_size=1)
#pub_back = rospy.Publisher("/BugaSegm/pc_segm_back", PointCloud2, queue_size=1)

#front_cloud = None
top_cloud = None
#back_cloud = None

#front_header = None
top_header = None
#back_header = None

'''def PredictionToImage(Prediction):
    # Map masterproject classes to kitti classes 
    Prediction = ProjectToKitti_LUT[Prediction]
    # Map kitti classes to colors
    Image = KittiToColor_LUT[Prediction]
    Image = np.swapaxes(Image,0,1)
    Image = Image[...,[2,1,0]]
    return Image'''

def predict(): 
    #global front_cloud
    global top_cloud
    #global back_cloud
    #global front_header
    global top_header
    #global back_header
    
    rospy.loginfo("LiDAR_Net - predict")
    
    mutex.acquire() 
    if (top_cloud is None):#front_cloud is None or top_cloud is None or back_cloud is None):
        mutex.release()
        return
     
    rospy.loginfo("LiDAR_Net - All topics received, start processing!")
        
    #front_cloud_exp = np.expand_dims(front_cloud, axis=0)
    top_cloud_exp = np.expand_dims(top_cloud, axis=0)
    #back_cloud_exp = np.expand_dims(back_cloud, axis=0)
    
    #batch_cloud = np.concatenate((front_cloud_exp, top_cloud_exp, back_cloud_exp), axis=0)
    batch_cloud = top_cloud_exp
                
    Prediction = model.predict(batch_cloud)
        
    Prediction = np.argmax(Prediction,axis=3)
        
    #front_predict = Prediction[0,:,:]
    top_predict = Prediction[0,:,:]
    #back_predict = Prediction[2,:,:]
    
    #plt.imsave('../data/images/front_predict_BugaLogImage.png', PredictionToImage(front_predict))
    #plt.imsave('../data/images/top_predict_BugaLogImage.png', PredictionToImage(top_predict))
    #plt.imsave('../data/images/back_predict_BugaLogImage.png', PredictionToImage(back_predict))

    # Translate back to sensor height
    #front_cloud[:,:,2] -= front['z'] - trained['z']
    top_cloud[:,:,2] -= top['z'] - trained['z']
    #back_cloud[:,:,2] -= back['z'] - trained['z']    
    
    if (args.layers == 'xyzi' or args.layers == 'xyzir'):
        #front_cloud[:,:,3]=front_predict
        top_cloud[:,:,3]=top_predict
        #back_cloud[:,:,3]=back_predict
    elif (args.layers == 'ir'):
        print("not implemented yet")
    elif (args.layers == 'xyz'):
        #front_predict = np.expand_dims(front_predict, axis=2)
        top_predict = np.expand_dims(top_predict, axis=2)
        #back_predict = np.expand_dims(back_predict, axis=2)
        #front_cloud = np.append(front_cloud, front_predict, axis=2).astype(np.float32)
        top_cloud = np.append(top_cloud, top_predict, axis=2).astype(np.float32)
        #back_cloud = np.append(back_cloud, back_predict, axis=2).astype(np.float32)
    
    #front_cloud = np.reshape(front_cloud, (front_cloud.shape[0]*front_cloud.shape[1], 4))
    top_cloud = np.reshape(top_cloud, (top_cloud.shape[0]*top_cloud.shape[1], 4))
    #back_cloud = np.reshape(back_cloud, (back_cloud.shape[0]*back_cloud.shape[1], 4))
    
    # Remove image coordinates where no point was projected onto
    #front_cloud = front_cloud[front_cloud[:,1]!= -1]
    top_cloud = top_cloud[top_cloud[:,1]!= -1]
    #back_cloud = back_cloud[back_cloud[:,1]!= -1]
    
    rospy.loginfo("LiDAR_Net - Publish new PointCloud on /BugaSegm/pc_segm_front")
    #pub_front.publish(array_to_PointCloud2(front_cloud, front_header))    
    pub_top.publish(array_to_PointCloud2(top_cloud, top_header))
    #pub_back.publish(array_to_PointCloud2(back_cloud, back_header))
    
    #front_cloud = None
    top_cloud = None
    #back_cloud = None
    
    #front_header = None
    top_header = None
    #back_header = None
    
    mutex.release()

'''def veloFrontCallback(data):
    global front_cloud
    global front_header
    rospy.loginfo("LiDAR_Net - Received new PointCloud on /BugaSegm/synchronized/velodyne/front/velodyne_points")
    front_header = data.header
    
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],4))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    
    # Rotate pointcloud to base
    front_rotation_matrix = R.from_euler('xyz', [front['roll'], front['pitch'], front['yaw']], degrees=False)
    points[:,0:3] = np.transpose(np.matmul(front_rotation_matrix.as_matrix(), np.transpose(points[:,0:3])))
    
    # Project points
    PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)
    front_cloud = getSampleArrayFromPointCloud_pcd(PointCloud, points, args.layers)
    
    # Translate to trained height
    if(args.layers != 'ir'):
        front_cloud[:,:,2] += front['z'] - trained['z']
    
    predict()'''

def veloTopCallback(data):
    global top_cloud
    global top_header
    rospy.loginfo("LiDAR_Net - Received new PointCloud on /BugaSegm/synchronized/velodyne/top/velodyne_points")
    top_header = data.header
    
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],4))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    
    # Rotate pointcloud to base
    top_rotation_matrix = R.from_euler('xyz', [top['roll'], top['pitch'], top['yaw']], degrees=False)
    points[:,0:3] = np.transpose(np.matmul(top_rotation_matrix.as_matrix(), np.transpose(points[:,0:3])))
    
    
    PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)
    top_cloud = getSampleArrayFromPointCloud_pcd(PointCloud, points, args.layers)
    
    # Translate to trained height
    if(args.layers != 'ir'):
        top_cloud[:,:,2] += top['z'] - trained['z']
    
    predict()

'''def veloBackCallback(data):
    global back_cloud
    global back_header
    rospy.loginfo("LiDAR_Net - Received new PointCloud on /BugaSegm/synchronized/velodyne/back/velodyne_points")
    back_header = data.header
    
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],4))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    
    # Rotate pointcloud to base
    back_rotation_matrix = R.from_euler('xyz', [back['roll'], back['pitch'], back['yaw']], degrees=False)
    points[:,0:3] = np.transpose(np.matmul(back_rotation_matrix.as_matrix(), np.transpose(points[:,0:3])))    
    
    PointCloud = SemLaserScan(20, KittiToColorDict, project=True, W=1440, H=16, fov_up=15, fov_down=-15.0)
    back_cloud = getSampleArrayFromPointCloud_pcd(PointCloud, points, args.layers)
    
    # Translate to trained height
    if(args.layers != 'ir'):
        back_cloud[:,:,2] += back['z'] - trained['z']
    
    predict()'''

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
    
    # Load params
    '''front_velo_to_base = rospy.get_param("/BASE_TO_FRONT")
    front ={'x': front_velo_to_base[0],
            'y': front_velo_to_base[1],
            'z': front_velo_to_base[2],
            'roll': front_velo_to_base[3],
            'pitch': front_velo_to_base[4],
            'yaw': front_velo_to_base[5],}'''
            
    top_velo_to_base = rospy.get_param("/BASE_TO_TOP")
    top ={'x': top_velo_to_base[0],
            'y': top_velo_to_base[1],
            'z': top_velo_to_base[2],
            'roll': top_velo_to_base[3],
            'pitch': top_velo_to_base[4],
            'yaw': top_velo_to_base[5],}
            
    '''back_velo_to_base = rospy.get_param("/BASE_TO_BACK")
    back ={'x': back_velo_to_base[0],
            'y': back_velo_to_base[1],
            'z': back_velo_to_base[2],
            'roll': back_velo_to_base[3],
            'pitch': back_velo_to_base[4],
            'yaw': back_velo_to_base[5],}'''
            
    trained_to_base = rospy.get_param("/BASE_TO_TRAINED")
    trained ={'x': trained_to_base[0],
            'y': trained_to_base[1],
            'z': trained_to_base[2],
            'roll': trained_to_base[3],
            'pitch': trained_to_base[4],
            'yaw': trained_to_base[5],}
    #start ros-listener
    #listener()
            
    #rospy.Subscriber('/BugaSegm/synchronized/velodyne/front/velodyne_points', PointCloud2, veloFrontCallback)
    rospy.Subscriber('/BugaSegm/synchronized/velodyne/top/velodyne_points', PointCloud2, veloTopCallback)
    #rospy.Subscriber('/BugaSegm/synchronized/velodyne/back/velodyne_points', PointCloud2, veloBackCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()