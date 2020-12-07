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

    #maxObj = (int)(points[:,4].max())

    #print("++++++++++++++++++++++++++++++MAX OBJ: ", maxObj)
    # filter points if distance > 20 meter, dont use them for other calculations anymore
    points = points[points[:,0] < 20]

    #maxClass = (int)(points[:,4].max())

    #print("++++++++++++++++++++++++++++++MAX OBJ: ", maxObj)
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
        xMinMaxMean.min = np.min(X)#X.min()
        xMinMaxMean.max = np.max(X)#X.max()
        xMinMaxMean.mean = np.mean(X)#X.mean()

        Y = objPoints[:,1]
        yMinMaxMean = MinMaxMean()
        yMinMaxMean.min = np.min(Y)#Y.min()
        yMinMaxMean.max = np.max(Y)#Y.max()
        yMinMaxMean.mean = np.mean(Y)#Y.mean()

        Z = objPoints[:,2]
        zMinMaxMean = MinMaxMean()
        zMinMaxMean.min = np.min(Z)#Z.min()
        zMinMaxMean.max = np.max(Z)#Z.max()
        zMinMaxMean.mean = np.mean(Z)#Z.mean()

        #Distance calculation
        dist = np.sqrt(np.power(X, 2)+np.power(Y, 2))
        distMinMaxMean = MinMaxMean()
        distMinMaxMean.min = np.min(dist)#dist.min()
        distMinMaxMean.max = np.max(dist)#dist.max()
        distMinMaxMean.mean = np.mean(dist)#dist.mean()

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
