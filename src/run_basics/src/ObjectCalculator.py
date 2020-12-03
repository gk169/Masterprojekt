#!/usr/bin/python3

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

    # filter points if distance > 20 meter, dont use them for other calculations anymore
    points = points[points[:,0] < 20]

    maxObj = (int)(points[:,4].max())

    fullObjectList = ObjectList()
    for objNr in range(1,maxObj+1):            #first object ist always irrelevant, sent from segm2object like that
        #print("OBJECT NUMBER: ", objNr)
        index = np.where(points[:,4] == objNr)
        #print(index[0].shape)
        objPoints = points[index]
        #print(objPoints.shape)

        if (0 < objPoints.shape[0]):
            # Filter x values with histogram from nearest to peak + 20%
            Histogram, Bins = np.histogram(objPoints[:,0], bins=10)
            HighestPeak = np.argmax(Histogram)
            NrOfBins = 2
            LowerBoarder = 0#HighestPeak-NrOfBins if HighestPeak-NrOfBins > 0 else 0
            UpperBoarder = HighestPeak+NrOfBins+1 if HighestPeak+NrOfBins+1 < 10 else 10
            objPoints = objPoints[objPoints[:,0] > Bins[LowerBoarder]]
            objPoints = objPoints[objPoints[:,0] < Bins[UpperBoarder]]

            # Filter y values with histogram (peak bin + 2 bins lower + 2 bins higher -> range of values = up to 50%, values probably more than 50% (because not equaly distributed)
            Histogram, Bins = np.histogram(objPoints[:,1], bins=10)
            HighestPeak = np.argmax(Histogram)
            NrOfBins = 2
            LowerBoarder = HighestPeak-NrOfBins if HighestPeak-NrOfBins > 0 else 0
            UpperBoarder = HighestPeak+NrOfBins+1 if HighestPeak+NrOfBins+1 < 10 else 10
            objPoints = objPoints[objPoints[:,1] > Bins[LowerBoarder]]
            objPoints = objPoints[objPoints[:,1] < Bins[UpperBoarder]]

            # Filter z values with std deviation; include +-2 sigma => 95%
            Z = objPoints[:,2]
            meanZ = np.mean(Z) #Z.mean()
            stdevZ = Z.std()
            objPoints = objPoints[abs(objPoints[:,2] - meanZ) < 2*stdevZ]
            
            #print(objPoints.shape)

            if (0 < objPoints.shape[0]):
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

    pub.publish(fullObjectList)

    rospy.loginfo("ObjectCalculator - Setting SEG_RUNNING to False")
    rospy.set_param('SEG_RUNNING', False)

def listener():
    rospy.Subscriber("/BugaSegm/pc_segmented", PointCloud2, callbackVelo)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
