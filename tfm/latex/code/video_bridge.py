#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

rospy.init_node("victure_cam")
img_pub = rospy.Publisher("victure_cam/image_raw", Image)
bridge = CvBridge()

cap = cv2.VideoCapture("/dev/video0")
while(True):
    ret, frame = cap.read()
    if ret:
        img_pub.publish(bridge.cv2_to_imgmsg(frame))

cap.release() # When everything done, release the capture
