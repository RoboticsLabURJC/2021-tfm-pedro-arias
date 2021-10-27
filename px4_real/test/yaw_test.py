#! /usr/bin/env python

import rospy
import time
from mavros_msgs.srv import CommandBool, CommandTOL, CommandTOLRequest
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

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

    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_pose_cb)
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    tk_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    print("Ready!")
    time.sleep(2)

    print("\nTaking off..", lat, lon, 1)
    arm_srv(True)
    resp = tk_srv(0, 0, lat, lon, 1)
    print(resp)
    time.sleep(10)


    # NOT WORKING
    setpoint = PositionTarget()
    setpoint.coordinate_frame = 8
    setpoint.type_mask = 1991
    setpoint.yaw_rate = 1
    n = 0
    while n < 9:
        vel_pub.publish(setpoint)
        n += 0.1
        time.sleep(0.1)
    #time.sleep(10)

    print("\nLanding..")
    resp = land_srv(0, 0, lat, lon, 0)
    print(resp)
    time.sleep(5)


if __name__ == "__main__":
    main()
