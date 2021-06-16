#! /usr/bin/env python

import rospy
from drone_wrapper import DroneWrapper
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np


RATE = 50


# ====== P CONTROLLER =======
def p_controller(Kp, error):
    up = Kp * error
    return up


# ====== D CONTROLLER =======
error_prev = 0
def d_controller(Kd, error):
    global error_prev
    de = error - error_prev
    error_prev = error
    ud = Kd * de
    return ud

error_prev_2 = 0
def d_controller_2(Kd, error):
    global error_prev_2
    de = error - error_prev_2
    error_prev_2 = error
    ud = Kd * de
    return ud


def execute(img):
    center_image_x = 960 / 2
    center_image_y = 720 / 2
    target_radius = 80

    red_low = np.array([0, 70, 50])
    red_high = np.array([5, 255, 255])
    red_low_2 = np.array([170, 70, 50])
    red_high_2 = np.array([180, 255, 255])

    rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    tmp_img = cv2.GaussianBlur(rgb_img, (11, 11), 0)
    imageHSV = cv2.cvtColor(tmp_img, cv2.COLOR_BGR2HSV)  # convert rgb space to hsv
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
            x_error = (int(radius) - target_radius)/target_radius

            # (center_image_x, center_image_y)
            cv2.arrowedLine(tmp_img, (int(x), int(y)), (center_image_x + yaw_error, center_image_y),
                            (255, 0, 0), thickness=3)
            cv2.arrowedLine(tmp_img, (int(x), int(y)), (center_image_x, center_image_y - z_error),
                            (0, 255, 0), thickness=3)
            cv2.circle(tmp_img, (int(x), int(y)), abs(x_error)*10, (0, 0, 255), cv2.FILLED, 4)
            if x_error > 0:
                cv2.putText(tmp_img, "X", (int(x)-10, int(y)+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
            else:
                cv2.circle(tmp_img, (int(x), int(y)), abs(x_error)*5, (0, 0, 0), cv2.FILLED, 4)

        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(tmp_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow("Cam", tmp_img)
    # cv2.imshow("BW", im2)
    cv2.waitKey(3)

    yaw_rate = p_controller(0.002, yaw_error) + d_controller(0.0001, yaw_error)
    vz = p_controller(0.0015, z_error)
    vx = -p_controller(0.2, x_error) + d_controller(0.0002, x_error)
    return -vx, 0, vz, yaw_rate
    # return 0, 0, 0, 0


def main():
    drone = DroneWrapper()

    drone.takeoff(h=1, precision=0.5)
    print("Taken off")

    rate = rospy.Rate(RATE)  # 50hz
    while not rospy.is_shutdown():
        try:
            img = drone.get_frontal_image()
            vx, vy, vz, yaw_rate = execute(img)
            drone.set_cmd_vel(vy, vx, vz, yaw_rate)
        except CvBridgeError:
            print("Error")
            pass

        rate.sleep()  # sleeps (50Hz)

    drone.land()


if __name__ == "__main__":
    main()
