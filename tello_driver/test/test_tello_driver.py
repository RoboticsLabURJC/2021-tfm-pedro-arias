#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandTOL
import time

ns_takeoff = 'mavros/cmd/takeoff'
ns_land = 'mavros/cmd/land'

try:
    rospy.wait_for_service(ns_takeoff, timeout=5)
    takeoff_srv = rospy.ServiceProxy(ns_takeoff, CommandTOL)
    print("Takeoff service ready!")
except rospy.ServiceException as e:
    print(e)

try:
    rospy.wait_for_service(ns_land, timeout=5)
    land_srv = rospy.ServiceProxy(ns_land, CommandTOL)
    print("Land service ready!")
except rospy.ServiceException as e:
    print(e)

resp = takeoff_srv(0.0, 0.0, 0.0, 0.0, 0.0)
print("Taking off!", resp)

time.sleep(5)

resp = land_srv(0.0, 0.0, 0.0, 0.0, 0.0)
print("Landing!", resp)
