#!/usr/bin/python3

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from run_basics.msg import MinMaxMean, singleObject, ObjectList
from sklearn.cluster import DBSCAN

rospy.init_node('ObjectCalculator', anonymous=True)
pub = rospy.Publisher("/BugaSegm/objectlist", ObjectList, queue_size=1)

def callbackVelo(data):
    rospy.loginfo("ObjectCalculator - Received new PointCloud on /BugaSegm/pc_segmented")

    pc = ros_numpy.numpify(data)

    points=np.zeros((pc.shape[0],5))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    points[:,4]=pc['ring']

    # filter points if distance > 20 meter, dont use them for other calculations anymore
    points = points[points[:,0] < 20]

    fullObjectList = ObjectList()

    #Use Segnet classes (except Sky) and LiDAR not road where Segnet is road.
    relevant_classes =  list(range(1,12))
    #relevant_classes.append(13) #add LiDAR_Objects
    relevant_classes.append(17)
    
    index = np.where(np.isin(points[:,3], relevant_classes))
    relevantPoints = points[index]
    db_points = DBSCAN(eps=0.75, min_samples=3).fit(relevantPoints[:,0:3])
    relevantPoints[:,4] = db_points.labels_
    
    maxObj = (int)(db_points.labels_.max())
    for objNr in range(maxObj+1):
        index_obj = np.where(relevantPoints[:,4] == objNr)
        objPoints = relevantPoints[index_obj]
        
	   #Dimension calculation
        X = objPoints[:,0]
        xMinMaxMean = MinMaxMean()
        xMinMaxMean.min = np.min(X)
        xMinMaxMean.max = np.max(X)
        xMinMaxMean.mean = np.mean(X)

        Y = objPoints[:,1]
        yMinMaxMean = MinMaxMean()
        yMinMaxMean.min = np.min(Y)
        yMinMaxMean.max = np.max(Y)
        yMinMaxMean.mean = np.mean(Y)

        Z = objPoints[:,2]
        zMinMaxMean = MinMaxMean()
        zMinMaxMean.min = np.min(Z)
        zMinMaxMean.max = np.max(Z)
        zMinMaxMean.mean = np.mean(Z)

        #Distance calculation
        dist = np.sqrt(np.power(X, 2)+np.power(Y, 2))
        distMinMaxMean = MinMaxMean()
        distMinMaxMean.min = np.min(dist)
        distMinMaxMean.max = np.max(dist)
        distMinMaxMean.mean = np.mean(dist)

        currentObject = singleObject()
        currentObject.x = xMinMaxMean
        currentObject.y = yMinMaxMean
        currentObject.z = zMinMaxMean
        currentObject.distance = distMinMaxMean
        unique, counts = np.unique(objPoints[:,3], return_counts=True)
        max_class = unique[np.argmax(counts)]
        currentObject.classNr = (int)(max_class)

        fullObjectList.ObjectList.append(currentObject)

    pub.publish(fullObjectList)

    rospy.loginfo("ObjectCalculator - Added obj to list " + str(len(fullObjectList.ObjectList)) + " --> Setting SEG_RUNNING to False")
    rospy.set_param('SEG_RUNNING', False)

def listener():
    rospy.Subscriber("/BugaSegm/pc_combined", PointCloud2, callbackVelo)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
