#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose

code_live_flag = False


def gui_play_stop_cb(msg):
	global code_live_flag, code_live_timer
	if msg.data == True:
		if not code_live_flag:
			code_live_flag = True
			code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	else:
		if code_live_flag:
			code_live_flag = False
			code_live_timer.shutdown()


def set_image_filtered(img):
	gui_filtered_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))


def set_image_threshed(img):
	gui_threshed_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))


def execute(event):
	global drone
	img_frontal = drone.get_frontal_image()
	# img_ventral = drone.get_ventral_image()

	# Both the above images are cv2 images
	################# Insert your code here #################################

	redLow = np.array([100, 0, 0])
	redHigh = np.array([180, 255, 255])

	tmp_img = cv2.GaussianBlur(img_frontal, (11, 11), 0)
	imageHSV = cv2.cvtColor(tmp_img, cv2.COLOR_BGR2HSV)  # convert rgb space to hsv
	mask = cv2.inRange(imageHSV, redLow, redHigh)  # create the correct mask to obtain the line

	result_image = cv2.bitwise_and(imageHSV, imageHSV, mask=mask)
	gray_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray_image, 200, 255, 0)
	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	for contour in contours:
		x, y, w, h = cv2.boundingRect(contour)
		cv2.rectangle(tmp_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

	set_image_filtered(tmp_img)
	set_image_threshed(gray_image)

	#########################################################################


if __name__ == "__main__":
	drone = DroneWrapper()
	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image, queue_size = 1)
	gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image, queue_size = 1)
	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()
