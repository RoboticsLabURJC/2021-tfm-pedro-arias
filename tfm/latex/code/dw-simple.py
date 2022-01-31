#!/usr/bin/env python
from drone_wrapper import DroneWrapper
from time import sleep

drone = DroneWrapper()
drone.takeoff(h=2.5)
drone.set_cmd_vel(az=1) # spin
sleep(5) # wait a couple of seconds
drone.land()