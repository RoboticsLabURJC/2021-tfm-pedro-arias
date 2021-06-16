#! /usr/bin/env python

import os
import rospy
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from mavros_msgs.srv import CommandTOL
from cv_bridge import CvBridge

import cv2
import numpy as np

ns_takeoff = 'mavros/cmd/takeoff'
ns_land = 'mavros/cmd/land'
ns_setpoint = 'mavros/setpoint_raw/local'
ns_bat = 'mavros/battery'
ns_state = 'mavros/state'
ns_ext_state = 'mavros/extended_state'
ns_pose = 'mavros/local_position/pose'
ns_vel = 'mavros/local_position/velocity_body'
ns_global = 'mavros/global_position/global'
ns_cam = 'tello/cam_frontal/image_raw'


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


class FollowColor:
    ext_state_codes = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR   ", 3: "TAKEOFF  ", 4: "LANDING  "}
    TEMP = "{}\n" \
           "\n" \
           "###########################\n" \
           "#      TELLO STATUS       #\n" \
           "#                         #\n" \
           "#  MODE: {}       #\n" \
           "#  STATE: {}       #\n" \
           "#  BAT:   {:02d} %            #\n" \
           "#  Z:     {} m            #\n" \
           "#                         #\n" \
           "###########################\n"

    def __init__(self):
        try:
            rospy.wait_for_service(ns_takeoff, timeout=5)
            self.takeoff_srv = rospy.ServiceProxy(ns_takeoff, CommandTOL)
            print("Takeoff service ready!")
        except rospy.ServiceException as e:
            print(e)

        try:
            rospy.wait_for_service(ns_land, timeout=5)
            self.land_srv = rospy.ServiceProxy(ns_land, CommandTOL)
            print("Land service ready!")
        except rospy.ServiceException as e:
            print(e)

        self.setpoint_raw_publisher = rospy.Publisher(ns_setpoint, PositionTarget, queue_size=1)
        rospy.Subscriber(ns_bat, BatteryState, self.battery_cb)
        rospy.Subscriber(ns_state, State, self.state_cb)
        rospy.Subscriber(ns_ext_state, ExtendedState, self.ext_state_cb)
        rospy.Subscriber(ns_pose, PoseStamped, self.pose_cb)
        rospy.Subscriber(ns_vel, TwistStamped, self.vel_cb)
        rospy.Subscriber(ns_global, NavSatFix, self.global_cb)
        rospy.Subscriber(ns_cam, Image, self.cam_cb)
        self.bridge = CvBridge()

        self.bat_percent = 0
        self.state = State()
        self.ext_state = 0
        self.pose = PoseStamped()
        self.h = 0
        self.vel = TwistStamped()
        self.glob = NavSatFix()

        self.cont = 0

        self.yaw_rate = 0
        self.vz = 0
        self.vx = 0

    def battery_cb(self, msg):
        self.bat_percent = int(msg.percentage)

    def state_cb(self, msg):
        self.state = msg

    def ext_state_cb(self, msg):
        self.ext_state = int(msg.landed_state)

    def pose_cb(self, msg):
        self.pose = msg
        self.h = msg.pose.position.z

    def vel_cb(self, msg):
        self.vel = msg

    def global_cb(self, msg):
        self.glob = msg

    def print_status(self):
        os.system('clear')  # Elegant-less
        print(self.TEMP.format(self.cont, self.state.mode, self.ext_state_codes[self.ext_state],
                               self.bat_percent, self.h))
        self.cont += 1

    def cam_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        center_image_x = 960 / 2
        center_image_y = 720 / 2
        target_radius = 80

        red_low = np.array([0, 70, 50])
        red_high = np.array([5, 255, 255])
        red_low_2 = np.array([170, 70, 50])
        red_high_2 = np.array([180, 255, 255])

        tmp_img = cv2.GaussianBlur(img, (11, 11), 0)
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
        cv2.imshow("BW", im2)
        cv2.waitKey(3)

        self.yaw_rate = p_controller(0.002, yaw_error) + d_controller(0.0001, yaw_error)
        self.vz = p_controller(0.0015, z_error)
        self.vx = -p_controller(0.2, x_error) + d_controller(0.0002, x_error)

    def update_vel(self):
        msg = PositionTarget()
        msg.coordinate_frame = 21  # FLU
        msg.type_mask = 1991  # vx vy vz yaw_rate

        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = 0
        msg.velocity.x = self.vx
        msg.velocity.z = self.vz
        msg.yaw_rate = self.yaw_rate
        self.setpoint_raw_publisher.publish(msg)


def main():
    rospy.init_node("follow_color")
    fc = FollowColor()

    fc.takeoff_srv(0.0, 0.0, 0.0, 0.0, 0.0)

    rate = rospy.Rate(RATE)  # 50hz
    while not rospy.is_shutdown():
        fc.print_status()
        fc.update_vel()
        rate.sleep()  # sleeps (50Hz)


if __name__ == "__main__":
    main()
