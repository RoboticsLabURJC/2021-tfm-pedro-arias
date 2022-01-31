#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

rospy.init_node("victure_cam")
victure_topic = rospy.get_param("victure_topic", "victure_cam/image_raw")
img_pub = rospy.Publisher(victure_topic, Image, queue_size=10)
bridge = CvBridge()

cap = cv2.VideoCapture("/dev/video0")
while(True):
    ret, frame = cap.read()
    if ret:
        img_pub.publish(bridge.cv2_to_imgmsg(frame))
        # img_pub.publish(bridge.cv2_to_imgmsg(cv2.flip(frame,0), "bgr8"))
    
    if 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
