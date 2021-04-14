#! /usr/bin/env python

import rospy
import tellopy
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool, ParamGet, ParamSet
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from sensor_msgs.msg import BatteryState, NavSatFix, Image, NavSatStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3
from math import degrees

import sys
import cv2
import numpy


class TelloDriver(tellopy.Tello):
    def __init__(self):
        tellopy.Tello.__init__(self)

        # TODO units
        self.__x = 0  # cm
        self.__y = 0  # cm
        self.__h = 0  # m
        self.__yaw = 0  # degrees
        self.__vx = 0  # m/s
        self.__vy = 0  # m/s
        self.__vz = 0  # m/s
        self.__yaw_rate = 0  # degrees/s

        self.state_pub = rospy.Publisher('mavros/state', State, queue_size=10)
        self.ext_state_pub = rospy.Publisher('mavros/extended_state', ExtendedState, queue_size=10)
        self.pose_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=10)
        self.vel_body_pub = rospy.Publisher('mavros/local_position/velocity_body', TwistStamped, queue_size=10)
        self.global_pub = rospy.Publisher('mavros/global_position/global', NavSatFix, queue_size=10)

        self.img_pub = rospy.Publisher('tello/cam_frontal/image_raw', Image, queue_size=10)

        self.bat_status_pub = rospy.Publisher('mavros/battery', BatteryState, queue_size=10)  # EXTRA

        rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget, self.setpoint_cb)

        self.takeoff_srv = rospy.Service('mavros/cmd/takeoff', CommandTOL, self.tello_takeoff)  # EXTRA

        self.arm_srv = rospy.Service('mavros/cmd/arming', CommandBool, self.tello_arm)
        self.set_mode_srv = rospy.Service('mavros/set_mode', SetMode, self.tello_set_mode)
        self.land_srv = rospy.Service('mavros/cmd/land', CommandTOL, self.tello_land)
        self.param_set_srv = rospy.Service('mavros/param/set', ParamSet, self.tello_param_set)
        self.param_get_srv = rospy.Service('mavros/param/get', ParamGet, self.tello_param_get)

        rospy.on_shutdown(self.shutdown)

        try:
            self.connect()
            self.start_video()
            self.subscribe(self.EVENT_FLIGHT_DATA, self.handler)
            self.subscribe(self.EVENT_VIDEO_FRAME, self.video_handler)
            self.wait_for_connection(60.0)
        except Exception as ex:
            rospy.logerr(ex)
            print(ex)

    def tello_takeoff(self, req):
        # float32 min_pitch  # used by takeoff
        # float32 yaw
        # float32 latitude
        # float32 longitude
        # float32 altitude
        # ---
        # bool success
        # uint8 result
        self.takeoff()
        rospy.loginfo("Taking off!")
        return True, 0

    def tello_arm(self, req):
        # bool value
        # ----
        # bool success
        # uint8 result
        rospy.loginfo("Tello Arming")
        if req.value:
            tk_req = CommandTOL()
            success, result = self.tello_takeoff(tk_req)  # arming is actually taking off in OFFBOARD flight mode
            return success, result
        else:
            return False, 0

    def tello_set_mode(self, req):
        # uint8 base_mode
        # string custom_mode
        # ----
        # bool mode_sent
        return False

    def tello_land(self, req):
        self.land()
        rospy.loginfo("Landing!")
        return True, 0

    def tello_param_set(self, req):
        # string param_id
        # mavros_msgs/ParamValue value
        # ----
        # bool success
        # mavros_msgs/ParamValue value
        return False, ParamValue(integer=0, real=0.0)

    def tello_param_get(self, req):
        # string param_id
        # ----
        # bool success
        # mavros_msgs/ParamValue value
        return False, ParamValue(integer=0, real=0.0)

    def setpoint_cb(self, msg):
        # CONTROL TELLO

        # uint8 coordinate_frame
        # uint16 type_mask
        # geometry_msgs/Point position
        # geometry_msgs/Vector3 velocity
        # geometry_msgs/Vector3 acceleration_or_force
        # float32 yaw
        # float32 yaw_rate

        mask = msg.type_mask
        mask = "{0:b}".format(mask)
        if bool(mask[-1]):
            target_x = msg.position.x
            x = target_x - self.__x
            if x > 0:
                self.forward(abs(int(x*100)))  # cm
            elif x < 0:
                self.back(abs(int(x*100)))  # cm
            else:
                pass  # already at target pos
        elif bool(mask[-2]):
            target_y = msg.position.y
            y = target_y - self.__y
            if y > 0:
                self.right(abs(int(y*100)))  # cm
            elif y < 0:
                self.left(abs(int(y*100)))  # cm
            else:
                pass  # already at target pos
        elif bool(mask[-3]):
            target_z = msg.position.z
            z = target_z - self.__h
            if z > 0:
                self.up(abs(int(z*100)))  # cm
            elif z < 0:
                self.down(abs(int(z*100)))  # cm
            else:
                pass  # already at target pos
        elif bool(mask[-4]):
            self.set_pitch(msg.velocity.x*100)  # linear x (cm/s)
        elif bool(mask[-5]):
            self.set_roll(msg.velocity.y*100)  # linear y (cm/s)
        elif bool(mask[-6]):
            self.set_throttle(msg.velocity.z*100)  # linear z (cm/s)
        elif bool(mask[-7]):
            pass  # AFX not supported
        elif bool(mask[-8]):
            pass  # AFY not supported
        elif bool(mask[-9]):
            pass  # AFZ not supported
        elif bool(mask[-10]):
            target_yaw = degrees(msg.yaw)
            yaw = target_yaw - self.__yaw
            if yaw > 0:
                self.clockwise(abs(yaw))
            elif yaw < 0:
                self.counter_clockwise(abs(yaw))
            else:
                pass  # already at target yaw
        elif bool(mask[-11]):
            self.set_yaw(degrees(msg.yaw_rate))

    def handler(self, event, sender, data, **args):
        tello = sender
        if event is tello.EVENT_FLIGHT_DATA:
            # rospy.loginfo(data)

            bat_percent = float("%2d" % data.battery_percentage)
            bat_state = bool(data.battery_state)

            # print(bool(data.em_open))  # em_open ?
            is_flying = bool(data.em_sky)
            is_on_ground = bool(data.em_ground)

            north_speed = float(data.north_speed)/100  # m
            east_speed = float(data.east_speed)/100  # m
            vertical_speed = -float(data.ground_speed)/100  # m

            height = float(data.height)/100  # m

            self.__vx = north_speed
            self.__vy = east_speed
            self.__vz = vertical_speed
            # self.__x = self.right_y
            # self.__y = self.right_x
            # self.__z = self.left_y
            # self.__yaw = self.left_x
            self.__h = height

            # print(data.em_sky, data.em_ground, data.em_open)

            is_armed = True if is_flying else False
            # landed_state = 1 if is_on_ground else 2 if is_flying else 0
            # TODO: review why not on ground?
            landed_state = 1 if is_on_ground else 2 if is_flying else 1

            state = State(connected=True, armed=is_armed, guided=False, manual_input=False, mode="OFFBOARD",
                          system_status=0)
            self.state_pub.publish(state)

            ext_state = ExtendedState(vtol_state=0, landed_state=landed_state)
            self.ext_state_pub.publish(ext_state)

            # TODO update pos attributes
            pose = PoseStamped(pose=Pose(position=Point(x=self.__x, y=self.__y, z=height),
                                         orientation=Quaternion(x=float('nan'), y=float('nan'), z=float('nan'),
                                                                w=float('nan'))))
            self.pose_pub.publish(pose)

            twist = TwistStamped(twist=Twist(linear=Vector3(x=north_speed, y=east_speed, z=vertical_speed),
                                             angular=Vector3(x=float('nan'), y=float('nan'), z=float('nan'))))
            self.vel_body_pub.publish(twist)

            # Empty, global pos not known
            nav_sat = NavSatFix(status=NavSatStatus(status=-1, service=0), latitude=float('nan'),
                                longitude=float('nan'), altitude=float('nan'), position_covariance=[float('nan')]*9,
                                position_covariance_type=0)
            self.global_pub.publish(nav_sat)

            # bat = BatteryState(voltage=0.0, temperature=float('nan'), current=float('nan'), charge=float('nan'),
            #                    capacity=float('nan'), design_capacity=float('nan'), percentage=bat_percent,
            #                    power_supply_status=0, power_supply_health=0, power_supply_technology=0, present=True,
            #                    cell_voltage=[float('nan')], cell_temperature=[float('nan')], location="0",
            #                    serial_number="")
            bat = BatteryState(voltage=0.0, current=float('nan'), charge=float('nan'),
                               capacity=float('nan'), design_capacity=float('nan'), percentage=bat_percent,
                               power_supply_status=0, power_supply_health=0, power_supply_technology=0, present=True,
                               cell_voltage=[float('nan')], location="0",
                               serial_number="")
            self.bat_status_pub.publish(bat)

    def video_handler(self, event, sender, data, **args):
        cap = cv2.VideoCapture(data)
        fourcc = cv2.VideoWriter_fourcc('H', '2', '6', '4')

        cap.set(6, fourcc)
        cap.set(3, 1920)
        cap.set(4, 1080)
        cap.set(5, 30)

        # vid = cv2.VideoWriter('../test.avi', fourcc, 20.0, (640, 480))
        # print(vid.isOpened())  # returns false :(
        while cap.isOpened():
            print("bucle")
            ret, frame = cap.read()
            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            print(image)
            cv2.imshow("Image", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # if ret:
            #     # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            #     vid.write(frame)
            #
            #     cv2.imshow('window', frame)
            #
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         break

        cap.release()
        # vid.release()
        cv2.destroyWindow('window')

    def shutdown(self):
        self.quit()
        rospy.loginfo("Tello driver shutting down")


def main():
    rospy.init_node('tello_driver')

    try:
        TelloDriver()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
