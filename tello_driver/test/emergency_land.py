#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandTOL

ns_land = 'mavros/cmd/land'

rospy.init_node("emergency_land")

land_srv = rospy.ServiceProxy(ns_land, CommandTOL)
resp = land_srv(0.0, 0.0, 0.0, 0.0, 0.0)
print("Emergency land", resp.success, resp.result)
