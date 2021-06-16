#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image

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


# ====== P CONTROLLER =======
def p_controller(Kp, error):
    up = Kp * error
    return up


# ====== D CONTROLLER =======
def d_controller(Kd, error):
    global error_prev
    de = error - error_prev
    error_prev = error
    ud = Kd * de
    return ud


def d_controller_2(Kd, error):
    global error_prev_2
    de = error - error_prev_2
    error_prev_2 = error
    ud = Kd * de
    return ud


def execute(event):
    global drone
    img = drone.get_frontal_image()
    # img_ventral = drone.get_ventral_image()

    # Both the above images are cv2 images
    ################# Insert your code here #################################

    red_low = np.array([0, 70, 50])
    red_high = np.array([5, 255, 255])
    red_low_2 = np.array([170, 70, 50])
    red_high_2 = np.array([180, 255, 255])

    tmp_img = cv2.GaussianBlur(img, (11, 11), 0)
    imageHSV = cv2.cvtColor(tmp_img, cv2.COLOR_RGB2HSV)  # convert rgb space to hsv
    mask1 = cv2.inRange(imageHSV, red_low, red_high)  # create the correct mask to obtain the line
    mask2 = cv2.inRange(imageHSV, red_low_2, red_high_2)
    mask = mask1 + mask2

    result_image = cv2.bitwise_and(imageHSV, imageHSV, mask=mask)
    gray_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_image, 150, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    moms = [cv2.moments(c) for c in contours]
    areas = [cv2.contourArea(c) for c in contours]

    yaw_error = 0
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

            yaw_error = -(center_image_x - cx)  # error between the center of the image and the current position of the centroid
            z_error = (center_image_y - cy)
            x_error = (int(radius) - target_radius) / target_radius

            # (center_image_x, center_image_y)
            cv2.arrowedLine(tmp_img, (int(x), int(y)), (center_image_x + yaw_error, center_image_y), (255, 0, 0), thickness=3)
            cv2.arrowedLine(tmp_img, (int(x), int(y)), (center_image_x, center_image_y - z_error), (0, 255, 0), thickness=3)
            cv2.circle(tmp_img, (int(x), int(y)), abs(x_error) * 10, (0, 0, 255), cv2.FILLED, 4)
            if x_error > 0:
                cv2.putText(tmp_img, "X", (int(x) - 10, int(y) + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
            else:
                cv2.circle(tmp_img, (int(x), int(y)), abs(x_error) * 5, (0, 0, 0), cv2.FILLED, 4)

        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(tmp_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    set_image_filtered(tmp_img)
    set_image_threshed(im2)

    yaw_rate = p_controller(0.002, yaw_error) + d_controller(0.0001, yaw_error)
    vz = p_controller(0.0015, z_error)
    vx = -p_controller(0.2, x_error) + d_controller(0.0002, x_error)

    # print(vx, 0, vz, yaw_rate)
    drone.set_cmd_vel(0, -vx, vz, yaw_rate)

#########################################################################


if __name__ == "__main__":
    drone = DroneWrapper()
    rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
    gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image, queue_size=1)
    gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image, queue_size=1)
    code_live_flag = False

    max_width_image = 960
    max_height_image = 720
    center_image_x = int(max_width_image / 2)
    center_image_y = int(max_height_image / 2)
    target_radius = 80

    error_prev = 0.0
    error_prev_2 = 0.0

    code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
    code_live_timer.shutdown()
    while not rospy.is_shutdown():
        rospy.spin()
