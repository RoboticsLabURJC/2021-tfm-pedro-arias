#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool, ParamGet, ParamSet
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from sensor_msgs.msg import BatteryState, NavSatFix, Image, NavSatStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3
from cv_bridge import CvBridge
from math import degrees

import cv2

import threading
import socket


# TODO: safety land each 15sec without cmd
# TODO: timer pubs
class TelloDriver:
    CMD_ADDRESS = ('192.168.10.1', 8889)
    STATE_ADDRESS = ('0.0.0.0', 8890)
    VIDEO_ADDRESS = ('0.0.0.0', 11111)
    LOCAL_ADDRESS = ('', 9000)

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.__state_dict = {}
        self.__running = True
        self.__flight_mode = "MANUAL"  # app
        self.__is_armed = False  # TODO sobra? Si is_flying estoy armado
        self.__is_flying = False
        self.__is_stream = False

        # TODO units
        self.__x = 0  # cm
        self.__y = 0  # cm
        self.__h = 0  # m
        self.__yaw = 0  # degrees
        self.__vx = 0  # cm/s
        self.__vy = 0  # cm/s
        self.__vz = 0  # cm/s
        self.__yaw_rate = 0  # degrees/s

        self.state_pub = rospy.Publisher('mavros/state', State, queue_size=10)
        self.ext_state_pub = rospy.Publisher('mavros/extended_state', ExtendedState, queue_size=10)
        self.pose_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=10)
        self.vel_body_pub = rospy.Publisher('mavros/local_position/velocity_body', TwistStamped, queue_size=10)
        self.global_pub = rospy.Publisher('mavros/global_position/global', NavSatFix, queue_size=10)

        self.bridge = CvBridge()
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
            self.sock.bind(self.LOCAL_ADDRESS)
            print(self.connect())

            self.set_stream(on=True)

            self.data_handler = threading.Thread(target=self.rcv_data)
            self.data_handler.start()

            self.video_handler = threading.Thread(target=self.rcv_video)
            self.video_handler.start()
        except Exception as ex:
            rospy.logerr(ex)
            print(ex)

    def __send_cmd(self, cmd):
        sent = self.sock.sendto(cmd.encode(encoding="utf-8"), self.CMD_ADDRESS)
        if sent == "ok":
            return True
        else:
            return False

    def connect(self):
        if self.__send_cmd("command"):
            self.__flight_mode = "OFFBOARD"
            return True
        else:
            # TODO raise exception
            return False

    def set_stream(self, on=True):
        if on:
            if self.__send_cmd("streamon"):
                self.__is_stream = True
        else:
            if self.__send_cmd("streamoff"):
                self.__is_stream = False

    def emergency_stop(self):
        self.__send_cmd("emergency")

    def tello_takeoff(self, req):
        # float32 min_pitch  # used by takeoff
        # float32 yaw
        # float32 latitude
        # float32 longitude
        # float32 altitude
        # ---
        # bool success
        # uint8 result

        if self.__send_cmd("takeoff"):
            rospy.loginfo("Taking off!")
            self.__is_flying = True
            return True, 0
        else:
            return False, 1

    def tello_arm(self, req):
        # bool value
        # ----
        # bool success
        # uint8 result
        rospy.loginfo("Tello Arming")
        if req.value:
            self.__is_armed = True
            tk_req = CommandTOL()
            success, result = self.tello_takeoff(tk_req)  # arming is actually taking off in OFFBOARD flight mode
            return success, result
        else:
            self.__is_armed = False
            return True, 0

    def tello_set_mode(self, req):
        # uint8 base_mode
        # string custom_mode
        # ----
        # bool mode_sent
        return False

    def tello_land(self, req):
        if self.__send_cmd("land"):
            rospy.loginfo("Landing!")
            self.__is_flying = False

            cmd = CommandBool()
            cmd.value = False
            success, result = self.tello_arm(cmd)
            return success, result
        else:
            return False, 1

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
        # uint8 coordinate_frame
        # uint16 type_mask
        # geometry_msgs/Point position
        # geometry_msgs/Vector3 velocity
        # geometry_msgs/Vector3 acceleration_or_force
        # float32 yaw
        # float32 yaw_rate

        # TODO coordination frame
        mask = msg.type_mask
        mask = "{0:b}".format(mask)
        if bool(mask[-1]):
            target_x = msg.position.x
            x = target_x - self.__x
            if x > 0:
                self.__send_cmd("forward {}".format(abs(int(x*100))))  # cm
            elif x < 0:
                self.__send_cmd("back {}".format(abs(int(x*100))))  # cm
            else:
                pass  # already at target pos
        elif bool(mask[-2]):
            target_y = msg.position.y
            y = target_y - self.__y
            if y > 0:
                self.__send_cmd("right {}".format(abs(int(y*100))))  # cm
            elif y < 0:
                self.__send_cmd("left {}".format(abs(int(y*100))))  # cm
            else:
                pass  # already at target pos
        elif bool(mask[-3]):
            target_z = msg.position.z
            z = target_z - self.__h
            if z > 0:
                self.__send_cmd("up {}".format(abs(int(z*100))))  # cm
            elif z < 0:
                self.__send_cmd("down {}".format(abs(int(z*100))))  # cm
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
                self.__send_cmd("cw {}".format(abs(yaw)))  # degrees
            elif yaw < 0:
                self.__send_cmd("ccw {}".format(abs(yaw)))  # degrees
            else:
                pass  # already at target yaw
        elif bool(mask[-11]):
            self.set_yaw(degrees(msg.yaw_rate))

    def rcv_data(self):
        while self.__running:
            try:
                msg, _ = self.sock.recvfrom(1024)
                msg = msg.decode(encoding="utf-8")
                state = msg.strip().split(";")

                for field in state:
                    pair = field.split(":")
                    if len(pair) < 2:
                        continue

                    key = pair[0]
                    value = pair[1]

                    self.__state_dict[key] = value

                # print("states: {}".format(msg.decode(encoding="utf-8")))
            except Exception as err:
                print(err)

    def update_pubs(self):
        bat_percent = self.__state_dict["bat"]

        height = self.__state_dict["h"]

        pitch = self.__state_dict["pitch"]
        roll = self.__state_dict["roll"]
        yaw = self.__state_dict["yaw"]
        # TODO to quaternion

        vx = self.__state_dict["vgx"]
        vy = self.__state_dict["vgy"]
        vz = self.__state_dict["vgz"]

        ax = self.__state_dict["agx"]
        ay = self.__state_dict["agy"]
        az = self.__state_dict["agz"]

        is_armed = self.__is_armed
        # landed_state = 1 if is_on_ground else 2 if is_flying else 0
        landed_state = 2 if self.__is_flying else 1

        state = State(connected=self.__running, armed=is_armed, guided=False, manual_input=False,
                      mode=self.__flight_mode, system_status=0)
        self.state_pub.publish(state)

        ext_state = ExtendedState(vtol_state=0, landed_state=landed_state)
        self.ext_state_pub.publish(ext_state)

        pose = PoseStamped(pose=Pose(position=Point(x=float('nan'), y=float('nan'), z=height),
                                     orientation=Quaternion(x=float('nan'), y=float('nan'), z=float('nan'),
                                                            w=float('nan'))))
        self.pose_pub.publish(pose)

        twist = TwistStamped(twist=Twist(linear=Vector3(x=vx, y=vy, z=vz),
                                         angular=Vector3(x=float('nan'), y=float('nan'), z=float('nan'))))
        self.vel_body_pub.publish(twist)

        # Empty, global pos not known
        nav_sat = NavSatFix(status=NavSatStatus(status=-1, service=0), latitude=float('nan'),
                            longitude=float('nan'), altitude=float('nan'), position_covariance=[float('nan')] * 9,
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

    def rcv_video(self):
        address = self.VIDEO_ADDRESS[0] + ":" + str(self.VIDEO_ADDRESS[1])
        cap = cv2.VideoCapture("udp://@" + address)

        while self.__running:
            try:
                ret, frame = cap.read()
                if ret:
                    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    self.img_pub.publish(self.bridge.cv2_to_imgmsg(img))
            except Exception as err:
                print(err)

        cap.release()
        cv2.destroyAllWindows()

    def shutdown(self):
        if self.__is_flying:
            self.tello_land(CommandTOL())
        if self.__is_stream:
            self.set_stream(on=False)
        self.__running = False
        self.data_handler.join()
        self.video_handler.join()
        self.sock.close()
        rospy.loginfo("Tello driver shutting down")

    def __del__(self):
        self.shutdown()


def main():
    rospy.init_node('tello_driver')

    try:
        TelloDriver()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
