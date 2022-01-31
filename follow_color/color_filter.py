#! /usr/bin/env python

import cv2
import numpy as np


img = cv2.imread('ref.jpeg')
# cv2.imshow('image', img)

height, width, channels = img.shape
center_image_x = width / 2
center_image_y = height / 2
target_radius = 10

red_low = np.array([0, 70, 50])
red_high = np.array([5, 255, 255])
red_low_2 = np.array([170, 70, 50])
red_high_2 = np.array([180, 255, 255])

blur_img = cv2.GaussianBlur(img, (11, 11), 0)
cv2.imwrite('blur.jpg', blur_img)

imageHSV = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)  # convert rgb space to hsv
cv2.imwrite('hsv.jpg', imageHSV)

mask1 = cv2.inRange(imageHSV, red_low, red_high)  # create the correct mask to obtain the line
mask2 = cv2.inRange(imageHSV, red_low_2, red_high_2)
mask = mask1 + mask2

result_image = cv2.bitwise_and(imageHSV, imageHSV, mask=mask)
cv2.imwrite('bit.jpg', result_image)

gray_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY)
cv2.imwrite('gray.jpg', gray_image)

ret, thresh = cv2.threshold(gray_image, 150, 255, 0)
cv2.imwrite('thresh.jpg', thresh)

im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (0,255, 255), 3)

moms = [cv2.moments(c) for c in contours]
areas = [cv2.contourArea(c) for c in contours]

y_error = 0
z_error = 0
x_error = 0
if len(contours) > 0:
	i = np.argmax(areas)

	contour = contours[i]
	area = areas[i]
	mom = moms[i]

	if mom["m00"] > 0:
		(x, y), radius = cv2.minEnclosingCircle(contours[i])
		cx = int(mom["m10"] / mom["m00"])
		cy = int(mom["m01"] / mom["m00"])

		y_error = (center_image_x - cx)  # error between the center of the image and the current position of the centroid
		z_error = (center_image_y - cy)
		x_error = (int(radius) - target_radius)/target_radius

		cv2.arrowedLine(img, (int(cx), int(cy)), (int(cx), center_image_y), (255, 0, 0), thickness=3)
		cv2.arrowedLine(img, (int(cx), int(cy)), (center_image_x, int(cy)), (0, 255, 0), thickness=3)
		cv2.circle(img, (int(x), int(y)), int(radius), (0, 0, 255), cv2.LINE_4, 4)

		cv2.circle(img, (int(center_image_x), int(center_image_y)), 8, (0, 0, 0), cv2.FILLED, 4)


		#x, y, w, h = cv2.boundingRect(contour)
		#cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

cv2.imshow("img", img)
cv2.imwrite('out.jpg', img)

cv2.waitKey(0)
cv2.destroyAllWindows()





#     cv2.imshow("Cam", tmp_img)
#     # cv2.imshow("BW", im2)
#     cv2.waitKey(3)
