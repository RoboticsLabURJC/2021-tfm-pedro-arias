#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from drone_wrapper import DroneWrapper
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import yolo_utils
import math

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

error_prev_3 = 0
def d_controller_3(Kd, error):
    global error_prev_3
    de = error - error_prev_3
    error_prev_3 = error
    ud = Kd * de
    return ud


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def execute(img, label, points):
    height, width, channels = img.shape
    center_image_x = width / 2
    center_image_y = height / 2
    target_area = 1500

    rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    y_error = 0
    z_error = 0
    x_error = 0
    if label:
        area = ((points[2] - points[0]) * (points[3] - points[1]))/2
        cx = (abs(points[0]) + abs(points[2]))/2
        cy = (abs(points[1]) + abs(points[3]))/2
        # print(label, (cx, cy), area)

        y_error = -(center_image_x - cx)  # error between the center of the image and the current position of the centroid
        z_error = (center_image_y - cy)
        x_error = (int(area) - target_area)/target_area

        cv2.arrowedLine(rgb_img, (int(cx), int(cy)), (center_image_x + y_error, center_image_y),
            (255, 0, 0), thickness=3)
        cv2.arrowedLine(rgb_img, (int(cx), int(cy)), (center_image_x, center_image_y - z_error),
            (0, 255, 0), thickness=3)
        #cv2.circle(rgb_img, (int(cx), int(cy)), 3, (0, 0, 255), cv2.FILLED, 4)

        if x_error < 0:
            cv2.putText(rgb_img, "X", (int(cx)-10, int(cy)+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
        else:
            cv2.circle(rgb_img, (int(cx), int(cy)), abs(x_error)*5, (0, 0, 255), cv2.FILLED, 4)

    cv2.imshow("Cam", rgb_img)
    cv2.waitKey(3)

    yaw_rate = p_controller(0.005, y_error) + d_controller(0.001, y_error)
    vz = p_controller(0.02, z_error) + d_controller(0.001, y_error)
    vx = p_controller(0.1, x_error) + d_controller(0.001, y_error)

    if isclose(vx, 0.0, rel_tol=0.00001):
        vx = 0.0001
    if isclose(vz, 0.0, rel_tol=0.00001):
        vz = 0.001
    return 0, -vx, vz, -yaw_rate


def main():
    img_pub = rospy.Publisher('my_solution/cam_frontal/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    drone = DroneWrapper()
    yolo4 = yolo_utils.YOLOv4()

    # drone.takeoff(h=2, precision=0.2)
    print("Taken off")

    rate = rospy.Rate(RATE)  # 50hz
    while not rospy.is_shutdown():
        try:
            img = drone.get_frontal_image()

            ### TEMP
            rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            yolo4.detect_frame(rgb_img)
            img_pub.publish(bridge.cv2_to_imgmsg(rgb_img, 'bgr8'))
            ## cv2.imshow("Cam", rgb_img)
            ## cv2.waitKey(3)

            # label, confidence, points = yolo4.detect_object(img, 'person')
            # vx, vy, vz, yaw_rate = execute(img, label, points)
            # print(vx, vy, vz, yaw_rate)
            # drone.set_cmd_vel(vy, vx, vz, yaw_rate)
        except CvBridgeError:
            print("Error")
            pass

        rate.sleep()  # sleeps (50Hz)

    drone.land()


if __name__ == "__main__":
    main()
