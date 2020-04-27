#!/usr/bin/env python

import math
import rospy
import pcl
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

rospy.init_node('segm2cloud', anonymous=True)
pub = rospy.Publisher('/segm_velodyne_points', PointCloud2, queue_size=10)

bridge = CvBridge()
latest_image = None

def getYID(point):
    #print(point)
    b = 2*point[0]
    width = 480

    pb = b/width

    numPixel = point[1]/pb

    yID = numPixel+width/2

    return int(round(yID))

def getZID(point):
    #print(point)
    d = 2*math.cos(math.radians(25))*point[0]
    height = 360

    ph = d/height

    numPixel = point[2]/ph

    zID = numPixel+height/2

    return int(round(zID))


def callbackVelo(data):
    rospy.loginfo(rospy.get_caller_id() + "Received new PointCloud on /velodyne/front/velodyne_points")

    header = data.header
    global latest_image
    pc = ros_numpy.numpify(data)
    #print(data)
    #print(pc)

    points=np.zeros((pc.shape[0],4))#,dtype="float32")
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']

    #cloud = pcl.PointCloud_PointXYZI(np.array(points, dtype=np.float32))

    image = np.asarray([latest_image])
    print(image.shape)
    #print(cloud.width)
    #print(cloud.height)


    for point in points:
        #print(point)
        yID = getYID(point)
        #print(yID)
        zID = getZID(point)
        #print(zID)
        r = image[0, zID, yID, 0].astype('float64')
        g = image[0, zID, yID, 1].astype('float64')
        b = image[0, zID, yID, 2].astype('float64')
        #print(type(point[0]))
        #print(type(point[1]))
        #print(type(point[2]))
        #print(type(point[3]))
        #print(type(r))
        point[3] = r #55.0

    #print(points)


    cloud = pcl.PointCloud_PointXYZI(np.array(points, dtype=np.float32))

    cloud_arr = cloud.to_array()

    #cloud_arr = np.atleast_2d(points)

    #cloud_arr = points

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]

    print(cloud_arr.shape[0])
    print(cloud_arr.shape[1])

    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = fields #dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = False #all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()

    #cloud = pcl.PointCloud_PointXYZI(np.array(points, dtype=np.float32))
    pub.publish(cloud_msg)

def callbackImg(img):
    print("received new image")
    global latest_image
    latest_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

def listener():
    rospy.Subscriber("/cropped_velodyne_points", PointCloud2, callbackVelo)
    #rospy.Subscriber("/img_segm", Image, callbackImg)
    rospy.Subscriber("/image_publisher_1587416786996765622/image_raw", Image, callbackImg)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
