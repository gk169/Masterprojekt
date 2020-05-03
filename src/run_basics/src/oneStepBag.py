#!/usr/bin/env python

import os
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import CompressedImage
import rosbag

dir_path = os.path.dirname(os.path.realpath(__file__))

rospy.init_node('oneStepbag', anonymous=True)
pub_front = rospy.Publisher('/static_points_front', PointCloud2, queue_size=10)
pub_top = rospy.Publisher('/static_points_top', PointCloud2, queue_size=10)
pub_img = rospy.Publisher('/static_img_raw', CompressedImage, queue_size=10)


def echoTopic():
    print(dir_path)
    bag = rosbag.Bag(dir_path+'/../rosbags/_2019-09-29-16-48-19_68_manuelle_Fahrt2.bag')

    x_front = 0
    x_top = 0
    x_img = 0
    msg_front = None
    msg_top = None
    msg_img = None
    for topic, msg, t in bag.read_messages(topics=['/velodyne/front/velodyne_points', '/velodyne/top/velodyne_points', '/usb_cam/image_raw/compressed']):
        #print(msg)
        print(topic)
        print(t)
        if topic == '/velodyne/front/velodyne_points' and x_front < 1:
            x_front=x_front+1
            msg_front = msg
        if topic == '/velodyne/top/velodyne_points' and x_top < 1:
            x_top=x_top+1
            msg_top = msg
        if topic == '/usb_cam/image_raw/compressed' and x_img < 1:
            x_img=x_img+1
            msg_img = msg
        if x_front==1 and x_top==1 and x_img==1:
            break

    print("publishing in")
    print("/velodyne/front/velodyne_points --> /static_points_front")
    print("/velodyne/top/velodyne_points --> /static_points_top")
    print("/usb_cam/image_raw/compressed --> /static_img_raw")
    while(True):
        pub_front.publish(msg_front)
        pub_top.publish(msg_top)
        pub_img.publish(msg_img)



if __name__ == '__main__':
    echoTopic()
