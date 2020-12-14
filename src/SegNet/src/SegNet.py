#!/usr/bin/env python2

import numpy as np
import os.path
import argparse
import cv2
import sys

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

sys.path.append('/usr/local/lib/python2.7/site-packages')

# Import arguments
parser = argparse.ArgumentParser()
parser.add_argument('--model', type=str, required=True)
parser.add_argument('--weights', type=str, required=True)
parser.add_argument('--colours', type=str, required=True)
parser.add_argument('--caffe', type=str, required=True)
args, unknown = parser.parse_known_args()

# Make sure that caffe is on the python path:
sys.path.insert(0, args.caffe + 'python')
os.environ['GLOG_minloglevel'] = '2'
import caffe

caffe.set_mode_gpu()

net = caffe.Net(args.model,
                args.weights,
                caffe.TEST)

input_shape = net.blobs['data'].data.shape
output_shape = net.blobs['argmax'].data.shape

label_colours = cv2.imread(args.colours).astype(np.uint8)

# Initialize the CvBridge class
bridge = CvBridge()

#publisher for segmentation image
pub = rospy.Publisher('/BugaSegm/img_segm', Image, queue_size=1000)

def img_callback(data):
    
    rospy.loginfo("SegNet - Received new image")

    img_header = data.header
    
    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    #cv2.namedWindow("Original")
    #cv2.namedWindow("Input")
    cv2.namedWindow("SegNet")

    #start = time.time()
    original = cv_image #cv2.imread("/home/micha/SegNet/Pictures/street1.jpg")
    #frame = cv2.imread("/home/micha/SegNet/CamVid/test/0001TP_008550.png")

    #end = time.time()
    #print '%30s' % 'Grabbed camera frame in ', str((end - start)*1000), 'ms'

    #start = time.time()
    frame = cv2.resize(original, (input_shape[3],input_shape[2]))
    #frame = cv2.resize(original, (480,270))
    input_image = frame.transpose((2,0,1))
    # input_image = input_image[(2,1,0),:,:] # May be required, if you do not open your data with opencv
    input_image = np.asarray([input_image])
    #end = time.time()
    #print '%30s' % 'Resized image in ', str((end - start)*1000), 'ms'
    #print 'resized to: ', str(input_shape[3]), ' - ', str(input_shape[2])

    #start = time.time()
    out = net.forward_all(data=input_image)
    #end = time.time()
    #print '%30s' % 'Executed SegNet in ', str((end - start)*1000), 'ms'

    #start = time.time()
    segmentation_ind = np.squeeze(net.blobs['argmax'].data)
    segmentation_ind_3ch = np.resize(segmentation_ind,(3,input_shape[2],input_shape[3]))
    #segmentation_ind_3ch = np.resize(segmentation_ind,(3,270,480))
    segmentation_ind_3ch = segmentation_ind_3ch.transpose(1,2,0).astype(np.uint8)
    segmentation_rgb = np.zeros(segmentation_ind_3ch.shape, dtype=np.uint8)

    cv2.LUT(segmentation_ind_3ch,label_colours,segmentation_rgb)
    #Use interpolation=cv2.INTER_NEAREST to obtain an image with no newly generated classes (Interpolation zwischen zwei Klassengrenzen)
    segmentation_rgb = cv2.resize(segmentation_rgb, (480,270), interpolation=cv2.INTER_NEAREST)
    segmentation_ind_3ch = cv2.resize(segmentation_ind_3ch, (480,270), interpolation=cv2.INTER_NEAREST)
    #segmentation_rgb = segmentation_rgb.astype(float)/255
    #segmentation_rgb = segmentation_rgb.astype(np.uint8)/255

    #end = time.time()
    #print '%30s' % 'Processed results in ', str((end - start)*1000), 'ms\n'

    rospy.loginfo("SegNet - Publishing segm-image")

    image_message = bridge.cv2_to_imgmsg(segmentation_ind_3ch, encoding="bgr8")
    image_message.header = img_header
    #print("SegNet - Header:", image_message.header)
    pub.publish(image_message)

    #frame = cv2.resize(frame, (480,270))
    #cv2.imshow("Original", original)
    #cv2.imshow("Input", frame)
    cv2.imshow("SegNet", segmentation_rgb)

    key = cv2.waitKey(1)

    #cv2.destroyAllWindows()

def listener():

    rospy.init_node('SegNet', anonymous=True)

    rospy.Subscriber('/BugaSegm/img_raw', Image, img_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
