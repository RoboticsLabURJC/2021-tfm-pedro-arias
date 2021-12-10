#! /usr/bin/env python

import rospy
import time
from mavros_msgs.srv import CommandBool, CommandTOL, CommandTOLRequest, SetMode
from sensor_msgs.msg import NavSatFix

RATE = 50
lat = 0
lon = 0
alt = 0

def global_pose_cb(msg):
    global lat
    global lon
    global alt
    lat, lon, alt =  msg.latitude, msg.longitude, msg.altitude


def main():
    rospy.init_node("tk_test")
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=2)
        rospy.wait_for_service('/mavros/cmd/takeoff', timeout=2)
        rospy.wait_for_service('/mavros/cmd/land', timeout=2)
    except rospy.exceptions.ROSException:
        print("Mavros not available")
        exit(1)

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_pose_cb)
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    tk_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    print("Ready!")
    time.sleep(2)

    print("\nTaking off..", lat, lon, 2.5)
    arm_srv(True)
    #resp = mode_srv(base_mode=0, custom_mode='AUTO.TAKEOFF')
    #print(resp)
    resp = tk_srv(altitude=2.5)
    print(resp)
    time.sleep(10)

    print("\nLanding..")
    resp = land_srv(0, 0, lat, lon, 0)
    print(resp)
    time.sleep(5)


if __name__ == "__main__":
    main()
