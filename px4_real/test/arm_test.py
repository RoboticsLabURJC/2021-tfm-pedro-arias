#! /usr/bin/env python

import rospy
import time
from mavros_msgs.srv import CommandBool

RATE = 50


def main():
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=2)
    except rospy.exceptions.ROSException:
        print("Mavros not available")
        exit(1)
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    print("Ready to arm")
    time.sleep(2)

    print("\nArming..")
    resp = arm_service(True)
    print(resp)
    time.sleep(5)

    print("\nDisarming..")
    resp = arm_service(False)
    print(resp)
    time.sleep(5)


if __name__ == "__main__":
    main()
