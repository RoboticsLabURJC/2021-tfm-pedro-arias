#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

def callback(msg):
    bridge =  CvBridge()
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('frame', img)
    cv2.waitKey(2)

rospy.init_node("test_victure_cam")
rospy.Subscriber("victure_cam/image_raw", Image, callback)
rospy.spin()
