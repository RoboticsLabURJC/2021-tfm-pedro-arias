#! /usr/bin/env python

import rospy
import tellopy
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import BatteryState


RATE = 10
tello = tellopy.Tello()

def tello_takeoff():
    tello.takeoff()
    rospy.loginfo("Taking off!")

def tello_land():
    tello.land()
    rospy.loginfo("Landing!")

def handler(event, sender, data, **args):
    tello = sender
    if event is tello.EVENT_FLIGHT_DATA:
        rospy.loginfo(data)

def shutdown():
    tello.quit()
    rospy.loginfo("Tello driver shutting down")

def tello_driver_node():
    rospy.init_node('tello_driver')
    rospy.on_shutdown(shutdown)

    try:
        tello.subscribe(tello.EVENT_FLIGHT_DATA, handler)
        tello.connect()
        tello.wait_for_connection(60.0)
    except Exception as ex:
        rospy.logerr(ex)
        print(ex)

    bat_status_pub = rospy.Publisher('battery', BatteryState, queue_size=10)

    takeoff_serv = rospy.Service('cmd/takeoff', CommandTOL, tello_takeoff)
    land_srv = rospy.Service('cmd/land', CommandTOL, tello_land)

    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        # rospy.loginfo("Loop")
        rate.sleep()  # sleeping RATE (10Hz)

if __name__ == '__main__':
    try:
        tello_driver_node()
    except rospy.ROSInterruptException:
        pass
