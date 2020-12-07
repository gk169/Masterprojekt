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
    points = points[points[:,0] < 10]

    #maxClass = (int)(points[:,4].max())

    #print("++++++++++++++++++++++++++++++MAX OBJ: ", maxObj)
    fullObjectList = ObjectList()

    #Use Segnet classes (except Sky) and LiDAR not road where Segnet is road.
    relevant_classes =  list(range(1,12))
    relevant_classes.append(17)
    
    for classNr in relevant_classes:
        index = np.where(points[:,3] == classNr)
        classPoints = points[index]
        min_samples_class = 3 if classNr == 17 else 10
        
        if (0 < classPoints.shape[0]):
            db_points = DBSCAN(eps=1, min_samples=min_samples_class).fit(classPoints)
            classPoints[:,4] = db_points.labels_
            maxObj = (int)(db_points.labels_.max())
            for objNr in range(maxObj+1):
                index_obj = np.where(classPoints[:,4] == objNr)
                objPoints = classPoints[index_obj]
                
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
                currentObject.classNr = (int)(objPoints[0,3])

                fullObjectList.ObjectList.append(currentObject)
                print("++++++++++++++++++++++++++++++Added obj to list", len(fullObjectList.ObjectList))

    pub.publish(fullObjectList)

    rospy.loginfo("ObjectCalculator - Setting SEG_RUNNING to False")
    rospy.set_param('SEG_RUNNING', False)

def listener():
    rospy.Subscriber("/BugaSegm/pc_combined", PointCloud2, callbackVelo)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
