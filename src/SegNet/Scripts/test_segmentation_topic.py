import numpy as np
import matplotlib.pyplot as plt
import os.path
import scipy
import argparse
import math
import cv2
import sys
import time

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

sys.path.append('/usr/local/lib/python2.7/site-packages')
# Make sure that caffe is on the python path:
caffe_root = '/home/micha/Dokumente/SegNet/caffe-segnet/'
sys.path.insert(0, caffe_root + 'python')
import caffe

# Import arguments
parser = argparse.ArgumentParser()
parser.add_argument('--model', type=str, required=True)
parser.add_argument('--weights', type=str, required=True)
parser.add_argument('--colours', type=str, required=True)
args = parser.parse_args()

net = caffe.Net(args.model,
                args.weights,
                caffe.TEST)

#caffe.set_mode_gpu()

input_shape = net.blobs['data'].data.shape
output_shape = net.blobs['argmax'].data.shape

label_colours = cv2.imread(args.colours).astype(np.uint8)

# Initialize the CvBridge class
bridge = CvBridge()

#publisher for segmentation image
pub = rospy.Publisher('/img_segm', Image, queue_size=10)

def img_callback(data):

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv2.namedWindow("Original")
    cv2.namedWindow("Input")
    cv2.namedWindow("SegNet")

    start = time.time()
    original = cv_image #cv2.imread("/home/micha/SegNet/Pictures/street1.jpg")
    #frame = cv2.imread("/home/micha/SegNet/CamVid/test/0001TP_008550.png")

    end = time.time()
    print '%30s' % 'Grabbed camera frame in ', str((end - start)*1000), 'ms'

    start = time.time()
    frame = cv2.resize(original, (input_shape[3],input_shape[2]))
    #frame = cv2.resize(original, (480,270))
    input_image = frame.transpose((2,0,1))
    # input_image = input_image[(2,1,0),:,:] # May be required, if you do not open your data with opencv
    input_image = np.asarray([input_image])
    end = time.time()
    print '%30s' % 'Resized image in ', str((end - start)*1000), 'ms'
    print 'resized to: ', str(input_shape[3]), ' - ', str(input_shape[2])

    start = time.time()
    out = net.forward_all(data=input_image)
    end = time.time()
    print '%30s' % 'Executed SegNet in ', str((end - start)*1000), 'ms'

    start = time.time()
    segmentation_ind = np.squeeze(net.blobs['argmax'].data)
    segmentation_ind_3ch = np.resize(segmentation_ind,(3,input_shape[2],input_shape[3]))
    #segmentation_ind_3ch = np.resize(segmentation_ind,(3,270,480))
    segmentation_ind_3ch = segmentation_ind_3ch.transpose(1,2,0).astype(np.uint8)
    segmentation_rgb = np.zeros(segmentation_ind_3ch.shape, dtype=np.uint8)

    cv2.LUT(segmentation_ind_3ch,label_colours,segmentation_rgb)
    segmentation_rgb = cv2.resize(segmentation_rgb, (480,270))
    #segmentation_rgb = segmentation_rgb.astype(float)/255
    #segmentation_rgb = segmentation_rgb.astype(np.uint8)/255

    end = time.time()
    print '%30s' % 'Processed results in ', str((end - start)*1000), 'ms\n'

    image_message = bridge.cv2_to_imgmsg(segmentation_rgb, encoding="passthrough")
    pub.publish(image_message)

    #frame = cv2.resize(frame, (480,270))
    cv2.imshow("Original", original)
    cv2.imshow("Input", frame)
    cv2.imshow("SegNet", segmentation_rgb)

    key = cv2.waitKey(0)

    cv2.destroyAllWindows()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('img_segm', anonymous=True)

    rospy.Subscriber('/usb_cam/image_raw/raw', Image, img_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
