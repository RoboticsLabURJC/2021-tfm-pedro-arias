#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

RATE = 0.01
LIN_VEL = 0.1
ANG_VEL = 0.1
msg = """
Teleoperate Target!
---------------------------
Moving around:
        w                i
   a    s    d      j    k    l
        x                ,
w/x : increase/decrease linear Z velocity (~ 0.1)
a/d : increase/decrease angular velocity (~ 0.1)
i/, : increase/decrease linear X velocity (~ 0.1)
j/l : increase/decrease linear Y velocity (~ 0.1)
space key, s, k : force stop
CTRL-C to quit
"""

e = """
Invalid command
"""

status = """
{vx}\t{vy}\t{vz}\t{az}
"""


class Target:
    def __init__(self):
        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

    def _set_vel(self, vx=0.0, vy=0.0, vz=0.0, ax=0.0, ay=0.0, az=0.0):
        state = self.get_state("teleop_target", "")

        req = ModelState()
        req.model_name = "teleop_target"
        req.pose = state.pose
        req.twist.linear.x = vx
        req.twist.linear.y = vy
        req.twist.linear.z = vz
        req.twist.angular.x = ax
        req.twist.angular.y = ay
        req.twist.angular.z = az
        self.set_state(req)

    def update(self):
        self._set_vel(vx=self.vx, vy=self.vy, vz=self.vz, ax=self.ax, ay=self.ay, az=self.az)

    def reset(self):
        self._set_vel()

    def print_status(self):
        print(status.format(vx=str(self.vx), vy=str(self.vy), vz=str(self.vz), az=str(self.az)))


if __name__ == "__main__":
    target = Target()
    while not rospy.is_shutdown():
        try:
            cmd = input(msg).lower()
            if cmd == "w":
                target.vz += LIN_VEL
            elif cmd == "a":
                target.az += ANG_VEL
            elif cmd == "d":
                target.az -= ANG_VEL
            elif cmd == "x":
                target.vz -= LIN_VEL
            elif cmd == "i":
                target.vx += LIN_VEL
            elif cmd == "j":
                target.vy -= LIN_VEL
            elif cmd == "l":
                target.vy += LIN_VEL
            elif cmd == ",":
                target.vx -= LIN_VEL
            elif cmd == "s" or "k" or " ":
                target.vx = 0
                target.vy = 0
                target.vz = 0
                target.az = 0
            elif cmd == "00":
                target.reset()
            else:
                raise ValueError
        except ValueError:
            print(e)
        else:
            target.update()
            target.print_status()
        rospy.sleep(RATE)
