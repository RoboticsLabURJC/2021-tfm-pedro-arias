#! /usr/bin/env python

import sys
import rospy
from drone_wrapper import DroneWrapper
from cv_bridge import CvBridgeError

import time

RATE = 50


def test_1(drone):
    print("RUNNING ARMING TEST")
    drone.arm(True)
    time.sleep(5)
    drone.arm(False)
    time.sleep(2)


def test_2(drone):
    print("RUNNING TAKEOFF TEST")
    drone.takeoff(h=1, precision=0.2)
    print("Taken off")

    time.sleep(5)

    drone.land()
    time.sleep(2)


def test_3(drone):
    print("RUNNING YAW TEST")
    drone.takeoff(h=1, precision=0.2)
    print("Taken off")

    time.sleep(5)
    drone.set_cmd_vel(0, 0, 0, 0.5)
    time.sleep(5)

    drone.land()
    time.sleep(2)


def test_4(drone):
    drone.takeoff(h=1, precision=0.2)
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


def main():
    drone = DroneWrapper()
    
    if len(sys.argv) > 1:
        if sys.argv[1] == '1':
            test_1(drone)
        elif sys.argv[1] == '2':
            test_2(drone)
        elif sys.argv[1] == '3':
            test_3(drone)
        elif sys.argv[1] == '4':
            test_4(drone)
        else:
            print("Invalid argument. Try [1-4]")
    else:
        test_1(drone)


if __name__ == "__main__":
    main()
