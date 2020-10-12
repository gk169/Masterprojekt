#!/usr/bin/env python

import os
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import CompressedImage, CameraInfo
import rosbag

dir_path = os.path.dirname(os.path.realpath(__file__))

rospy.init_node('RosBagPlay', anonymous=True)
pub_front = rospy.Publisher('/velodyne/front/velodyne_points', PointCloud2, queue_size=10)
pub_top = rospy.Publisher('/velodyne/top/velodyne_points', PointCloud2, queue_size=10)
pub_back = rospy.Publisher('/velodyne/back/velodyne_points', PointCloud2, queue_size=10)
pub_img = rospy.Publisher('/usb_cam/image_raw/compressed', CompressedImage, queue_size=10)
pub_cam = rospy.Publisher('/usb_cam/camera_info', CameraInfo, queue_size=10)


def echoTopic():
    print(dir_path)
    bag = rosbag.Bag(dir_path+'/../rosbags/_2019-09-05-11-49-39_70.bag')

    #bag = rosbag.Bag(dir_path+'/../rosbags/_2019-09-29-16-48-19_68_manuelle_Fahrt2.bag')

    x_front = 0
    x_top = 0
    x_back = 0
    x_img = 0
    x_cam = 0
    msg_front = None
    msg_top = None
    msg_back = None
    msg_img = None
    msg_cam = None
    for topic, msg, t in bag.read_messages(topics=['/velodyne/front/velodyne_points', '/velodyne/top/velodyne_points', '/velodyne/back/velodyne_points', '/usb_cam/image_raw/compressed', '/usb_cam/camera_info']):
        #print(msg)
        #print(topic)
        #print(t)
        if topic == '/velodyne/front/velodyne_points' and x_front < 1250:
            x_front=x_front+1
            msg_front = msg
        if topic == '/velodyne/top/velodyne_points' and x_top < 1250:
            x_top=x_top+1
            msg_top = msg
        if topic == '/velodyne/back/velodyne_points' and x_back < 1250:
            x_back=x_back+1
            msg_back = msg
        if topic == '/usb_cam/image_raw/compressed' and x_img < 1250:
            x_img=x_img+1
            msg_img = msg
        if topic == '/usb_cam/camera_info' and x_cam < 1250:
            x_cam=x_cam+1
            msg_cam = msg
        if x_front==x_top==x_back==x_img==x_cam==1250:
            break

    print("publishing")
    print("/velodyne/front/velodyne_points")
    print("/velodyne/top/velodyne_points")
    print("/velodyne/back/velodyne_points")
    print("/usb_cam/image_raw/compressed")
    print("/usb_cam/camera_info")
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        pub_front.publish(msg_front)
        pub_top.publish(msg_top)
        pub_back.publish(msg_back)
        pub_img.publish(msg_img)
        pub_cam.publish(msg_cam)
        rate.sleep()



if __name__ == '__main__':
    try:
        echoTopic()
    except rospy.ROSInterruptException:
        pass
