#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from drone_wrapper import DroneWrapper
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import yolo_utils
import math
import time

ASPECT_RATIO = 30  # la trigesima parte de la imagen esta ocupada por el area de le persona
RATE = 50

yolo_pub = rospy.Publisher('/brain/yolo_output/image_raw', Image, queue_size=10)
cmd_pub = rospy.Publisher('/brain/cmd_response/image_raw', Image, queue_size=10)
bridge = CvBridge()


class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator = 0
        self.Derivator = 0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator


vx_pid = PID(P=rospy.get_param('/drone/vx/kp'),
             I=rospy.get_param('/drone/vx/ki'),
             D=rospy.get_param('/drone/vx/kd'))
Yr_pid = PID(P=rospy.get_param('/drone/yaw_rate/kp'),
             I=rospy.get_param('/drone/yaw_rate/ki'),
             D=rospy.get_param('/drone/yaw_rate/kd'))
vz_pid = PID(P=rospy.get_param('/drone/vz/kp'),
             I=rospy.get_param('/drone/vz/ki'),
             D=rospy.get_param('/drone/vz/kd'))


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def execute(img, label, points):
    height, width, channels = img.shape
    center_image_x = width / 2
    center_image_y = height / 2
    target_area = height * width / ASPECT_RATIO

    rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    x_error, yaw_error, z_error = 0, 0, 0
    if label:
        area = ((points[2] - points[0]) * (points[3] - points[1]))/2
        cx = (abs(points[0]) + abs(points[2]))/2
        cy = (abs(points[1]) + abs(points[3]))/2

        # error between the center of the image and the current position of the centroid
        x_error = (int(area) - target_area)/target_area
        yaw_error = (center_image_x - cx)
        z_error = (center_image_y - cy)

        cv2.arrowedLine(rgb_img, (int(cx), int(cy)), (int(cx), center_image_y), (255, 0, 0), thickness=3)
        cv2.arrowedLine(rgb_img, (int(cx), int(cy)), (center_image_x, int(cy)), (0, 255, 0), thickness=3)
        if x_error < 0:
            cv2.putText(rgb_img, "X", (int(cx)-10, int(cy)+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
        else:
            cv2.circle(rgb_img, (int(cx), int(cy)), abs(x_error)*3, (0, 0, 255), cv2.FILLED, 4)
    else:
        # Looking for person to follow
        return 0, 0, 0, 0.3

    cmd_pub.publish(bridge.cv2_to_imgmsg(rgb_img, 'bgr8'))

    print("Error", x_error, yaw_error, z_error)
    vx = vx_pid.update(x_error)
    # vy = 0
    yaw_rate = Yr_pid.update(yaw_error)
    vz = vz_pid.update(z_error)

    print("Vels", vx, yaw_rate, vz)

    if isclose(vx, 0.0, rel_tol=0.00001):
        vx = 0.0001
    if isclose(vz, 0.0, rel_tol=0.00001):
        vz = 0.001
    return vx, 0, vz, yaw_rate


def main():
    drone = DroneWrapper(name=rospy.get_param('/drone/model'))
    yolo4 = yolo_utils.YOLOv4()

    drone.takeoff(h=1, precision=0.2)
    print("Taken off")

    rate = rospy.Rate(RATE)  # 50hz

    # FPS
    n_frames = 0
    t0 = time.time()
    while not rospy.is_shutdown():
        try:
            img = drone.get_frontal_image()
            n_frames += 1
            rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            yolo4.detect_frame(rgb_img)
            yolo_pub.publish(bridge.cv2_to_imgmsg(rgb_img, 'bgr8'))

            label, confidence, points = yolo4.detect_object(img, 'person')
            vx, vy, vz, yaw_rate = execute(img, label, points)
            drone.set_cmd_vel(vx, vy, vz, yaw_rate)

            # Print FPS
            print("FPS:", n_frames/(time.time() - t0))
        except CvBridgeError:
            print("Error")
            pass

        rate.sleep()  # sleeps (50Hz)

    drone.land()


if __name__ == "__main__":
    main()
