import time
import math
import numpy as np
import os
from video_from_drone import VideoCapture, LandingTargetDetector
from scipy.spatial.transform import Rotation

from PyQt5.QtWidgets import QApplication
from views import StartWindow

# import os
# os.environ['MAVLINK20']='1'

from pymavlink import mavutil
import threading


class MavLinkMessageLogger():
    def __init__(self, master):
        self.master = master
        thread = threading.Thread(target=self.start, )
        self.continue_flag = True
        thread.start()

    def __del__(self):
        print("calling destructor")

    def start(self):
        while True:
            msg = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if msg is not None:
                print(msg)


class Navigator:
    def __init__(self,landing_target_data_stack):

        self.landing_target_data_stack = landing_target_data_stack

        print(">>CONNECTING TO DRONE<<")
        self.master = mavutil.mavlink_connection(
            "udp:192.168.1.30:14560")  # in Arducopter cmd type: output add 192.168.1.18:14560

        self.master.mav.ping_send(
            int(time.time() * 1e6),  # Unix time in microseconds
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        print(">>WAITING FOR HEARTBEAT<<")
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))
        MavLinkMessageLogger(self.master)

    def between_two_numbers(self, num, a, b):
        if b < a:
            a, b = b, a
        if a < num and num < b:
            return True
        else:
            return False


    def move_to_frd_ned(self, f, r, d):
        print("move NED")
        self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.master.target_system,
                                                                                      self.master.target_component,
                                                                                      mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                      int(0b000011111000), f, r, d, 0, 0, 0,
                                                                                      0, 0, 0, 0, 0))
        arrived = False
        while not arrived:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  #
            print(msg)
            if self.between_two_numbers(msg.x, f - .1, f + 0.1) and self.between_two_numbers(msg.y, r - .1,
                                                                                   r + 0.1) and self.between_two_numbers(msg.z,
                                                                                                                    d - .1,
                                                                                                                    d + 0.1):
                print("arrived")
                arrived = True


    def rotate_cc_to(self, rotation_z, direction):
        print("rotating")
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        start_yaw = math.degrees(msg.yaw)
        print("start yaw {}".format(start_yaw))
        target_yaw = start_yaw + direction * rotation_z
        if target_yaw > 180:
            target_yaw = target_yaw - 360
        if target_yaw < -180:
            target_yaw = target_yaw + 360
        print("target yaw {}".format(target_yaw))

        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                     int(rotation_z), 5, direction, 1, 0, 0, 0, 0)

        arrived = False
        while not arrived:
            msg = self.master.recv_match(type='ATTITUDE', blocking=True)  #
            current_yaw = math.degrees(msg.yaw)
            if current_yaw > 180:
                current_yaw = current_yaw - 360
            print(current_yaw)
            if self.between_two_numbers(current_yaw, target_yaw - 5, target_yaw + 5):
                print("arrived")
                arrived = True


    def land(self):
        print(">>>>>>>>>>LANDING<<<<<<<<<<<<<")
        self.cpt = 0
        arrived = False
        while not arrived:
            if len(self.landing_target_data_stack) > 0:
                # print(len(landing_target))
                data = self.landing_target_data_stack.pop()

                t_vec_big = data[0]
                t_vec_small = data[1]
                r_vec_big = data[2]
                r_vec_small = data[2]

                t_vec_used = None
                r_vec_used = None

                angle_rate_used = None
                using = "None"
                if t_vec_big is not None:
                    t_vec_used = t_vec_big
                    r_vec_used = r_vec_big
                    angle_rate_used = 5.0
                    using = "Big"
                if t_vec_small is not None:
                    t_vec_used = t_vec_small
                    r_vec_used = r_vec_small
                    angle_rate_used = 1.0
                    using = "Small"
                if t_vec_used is not None:
                    '''save working on aruco
                    height = tvec[0][0][2]
                    forward = 0.25-tvec[0][0][1]
                    right = tvec[0][0][0]+0.25
                    '''
                    height = t_vec_used[2][0]
                    forward = -t_vec_used[1][0]
                    right = t_vec_used[0][0]

                    if r_vec_used is not None:
                        r_vec_used = np.asarray(r_vec_used)
                        r = Rotation.from_matrix(r_vec_used)
                        angles = r.as_euler("xyz", degrees=True)
                        # quat = r.as_quat()
                        # print("rot {}".format(angles))
                        # print("{}: {},{},{}".format(using, forward, right, height))

                        self.master.mav.landing_target_send(
                            0,  # time_usec
                            1,  # target_num
                            mavutil.mavlink.MAV_FRAME_BODY_FRD,  # MAV Frame
                            0,  # angle_x
                            0,  # angle_y
                            height,  # distance height
                            0.5,  # size_x
                            0.5,  # size_y
                            forward,  # x** forward
                            right,  # y** right
                            height,  # z** height
                            [1, 0, 0, 0],  # q** no rotation [1,0,0,0] quat not having effect
                            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                            1)
                        self.cpt+=1
                        if self.cpt == 10:
                            yaw = angles[2]
                            direction = 1
                            if yaw < 0:
                                yaw = abs(yaw)
                                direction = -1


                            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                                         mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                                         int(yaw), angle_rate_used, direction, 1, 0, 0, 0, 0)
                            self.cpt=0

    def basic_nav(self):
        # move_to_frd_ned(0,0,-50)
        # Set all basic parameters for PRECISION LANDING TO work MAV_CMD_DO_SET_PARAMETER
        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                  b'LAND_SPEED', 30.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                  b'LAND_ALT_LOW', 1000.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                  b'LAND_SPEED_HIGH', 300.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        print("guided mode")
        # mode guide
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                     1, 4, 0, 0, 0, 0, 0, 0)
        print("arm")
        # ARM
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     1, 0, 0, 0, 0, 0, 0, 0)

        # takeoff 20
        print("send takeoff")
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                     0, 0, 0, 0, 0, 0, 2)
        cmd_accepted = False
        while cmd_accepted:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)  #
            print(msg)
        '''
        arrived = False
        while not arrived:
            msg = master.recv_match(type='LOCAL_POSITION_NED',blocking=True) #
            print(msg)
            if between_two_numbers(msg.z,-20.30,-19.70) :
                print("arrived")
                arrived = True
        '''
        print("Wait 10 sec")
        time.sleep(10)

        print("travel to NED dest.")
        self.move_to_frd_ned(1, 2, -20)

        print("rotate random degrees CCW")
        import random

        rot_rand = random.randint(0, 60)
        self.rotate_cc_to(rot_rand, 1)

        # set land mode
        print("Start landing")
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                     1, 9, 0, 0, 0, 0, 0, 0)

        self.land()


print("starting video")
images_stack = []
landing_target_data_stack = []
analyzed_img_stack = []
capture = VideoCapture(images_stack)
LandingTargetDetector = LandingTargetDetector(images_stack, landing_target_data_stack, analyzed_img_stack)
navigator = Navigator(landing_target_data_stack)

app = QApplication([])
start_window = StartWindow(analyzed_img_stack,navigator)
start_window.show()
app.exit(app.exec_())




