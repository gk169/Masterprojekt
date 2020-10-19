#!/usr/bin/env python

import math
import rospy
import pcl
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from run_basics.msg import MinMaxMean, singleObject, ObjectList

rospy.init_node('ObjectCalculator', anonymous=True)
pub = rospy.Publisher("/BugaSegm/objectlist", ObjectList, queue_size=1)

def callbackVelo(data):
    rospy.loginfo("ObjectCalculator - Received new PointCloud on /velodyne/front/segm_velodyne_points")

    pc = ros_numpy.numpify(data)
    #print(data)
    #print(pc)

    points=np.zeros((pc.shape[0],5))#,dtype="float32")
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    points[:,4]=pc['ring']

    #cloud = pcl.PointCloud_PointXYZI(np.array(points, dtype=np.float32))

    #print(points[:,3].max()) #class
    #print(points[:,4].max()) #object

    maxObj = (int)(points[:,4].max())

    fullObjectList = ObjectList()
    for objNr in range(1,maxObj+1):            #first object ist always irrelevant, sent from segm2object like that
        #print("OBJECT NUMBER: ", objNr)
        index = np.where(points[:,4] == objNr)
        #print(index[0].shape)
        objPoints = points[index]
        #print(objPoints.shape)
        if (0 < objPoints.shape[0]):

            X = objPoints[:,0]
            meanX = X.mean()
            stdevX = X.std()

            Y = objPoints[:,1]
            meanY = Y.mean()
            stdevY = Y.std()

            Z = objPoints[:,2]
            meanZ = Z.mean()
            stdevZ = Z.std()

            #print("X: ", X.max())
            #print("Y: ", Y.max())
            #print("Z: ", Z.max())

            objPoints = objPoints[abs(objPoints[:,0] - meanX) < 2*stdevX]
            #print(objPoints.shape)
            objPoints = objPoints[abs(objPoints[:,1] - meanY) < 2*stdevY]
            #print(objPoints.shape)
            objPoints = objPoints[abs(objPoints[:,2] - meanZ) < 2*stdevZ]
            #print(objPoints.shape)

            if (0 < objPoints.shape[0]):
	        #Dimension calculation
                X = objPoints[:,0]
                xMinMaxMean = MinMaxMean()
                xMinMaxMean.min = X.min()
                xMinMaxMean.max = X.max()
                xMinMaxMean.mean = X.mean()

                Y = objPoints[:,1]
                yMinMaxMean = MinMaxMean()
                yMinMaxMean.min = Y.min()
                yMinMaxMean.max = Y.max()
                yMinMaxMean.mean = Y.mean()

                Z = objPoints[:,2]
                zMinMaxMean = MinMaxMean()
                zMinMaxMean.min = Z.min()
                zMinMaxMean.max = Z.max()
                zMinMaxMean.mean = Z.mean()

                #Distance calculation
                dist = np.sqrt(np.power(X, 2)+np.power(Y, 2))
                distMinMaxMean = MinMaxMean()
                distMinMaxMean.min = dist.min()
                distMinMaxMean.max = dist.max()
                distMinMaxMean.mean = dist.mean()

                currentObject = singleObject()
                currentObject.x = xMinMaxMean
                currentObject.y = yMinMaxMean
                currentObject.z = zMinMaxMean
                currentObject.distance = distMinMaxMean
                currentObject.classNr = (int)(objPoints[0,3])

                fullObjectList.ObjectList.append(currentObject)

    pub.publish(fullObjectList)

    rospy.loginfo("ObjectCalculator - Setting SEG_RUNNING to False")
    rospy.set_param('SEG_RUNNING', False)

def listener():
    rospy.Subscriber("/BugaSegm/pc_segmented", PointCloud2, callbackVelo)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
