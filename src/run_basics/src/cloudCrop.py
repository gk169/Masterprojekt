#!/usr/bin/env python

import rospy
import pcl
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2

rospy.init_node('cloudCrop', anonymous=True)
pub = rospy.Publisher('/cropped_velodyne_points', PointCloud2, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Received new PointCloud on /velodyne/front/velodyne_points")

    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    cloud = pcl.PointCloud(np.array(points, dtype=np.float32))

    clipper = cloud.make_cropbox()

    # pcl::PCDWriter writer;
    # pcl::PointCloud<PointXYZI>::Ptr outcloud;
    outcloud = pcl.PointCloud()

    # clipper.setTranslation(Eigen::Vector3f(pose->tx, pose->ty, pose->tz));
    # clipper.setRotation(Eigen::Vector3f(pose->rx, pose->ry, pose->rz));
    # clipper.setMin(-Eigen::Vector4f(tracklet->l/2, tracklet->w/2, 0, 0));
    # clipper.setMax(Eigen::Vector4f(tracklet->l/2, tracklet->w/2, tracklet->h, 0));
    # clipper.filter(*outcloud);
    tx = 0
    ty = 0
    tz = 0
    clipper.set_Translation(tx, ty, tz)
    rx = 0
    ry = 0
    rz = 0
    clipper.set_Rotation(rx, ry, rz)
    minx = -1.5
    miny = -1.5
    minz = 2
    mins = 0
    maxx = 3.5
    maxy = 3.5
    maxz = 3
    maxs = 0
    clipper.set_MinMax(minx, miny, minz, mins, maxx, maxy, maxz, maxs)
    outcloud = clipper.filter()

#    msg = ros_numpy.msgify(PointCloud2, pc)

    cloud_arr = np.asarray(cloud)

    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()

    rospy.loginfo(rospy.get_caller_id() + "SEND new PointCloud")
    pub.publish(cloud_msg)

def listener():
    rospy.Subscriber("/velodyne/front/velodyne_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
