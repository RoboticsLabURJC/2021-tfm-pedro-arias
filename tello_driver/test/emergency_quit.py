#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool

ms_emergency = 'tello/emergency'

rospy.init_node("emergency_quit")

emergency_srv = rospy.ServiceProxy(ms_emergency, CommandBool)
resp = emergency_srv(True)
print("Emergency quit", resp.success, resp.result)
