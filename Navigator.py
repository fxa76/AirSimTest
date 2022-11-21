from scipy.spatial.transform import Rotation
import threading
import time
# import os
# os.environ['MAVLINK20']='1'
import queue

from pymavlink import mavutil
import threading
import numpy as np

class Navigator(threading.Thread):
    def __init__(self, continue_flag, landing_target_data_stack, textLogger):
        super(Navigator, self).__init__()
        self.continue_flag = continue_flag
        self.landing_target_data_stack = landing_target_data_stack
        self.textLogger = textLogger

        self.textLogger.log(">>CONNECTING TO DRONE<<")
        self.master = mavutil.mavlink_connection(
            "udp:192.168.1.30:14560")  # in Arducopter cmd type: output add 192.168.1.18:14560

        self.master.mav.ping_send(
            int(time.time() * 1e6),  # Unix time in microseconds
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        self.textLogger.log(">>WAITING FOR HEARTBEAT<<")
        self.master.wait_heartbeat()
        self.textLogger.log("Heartbeat from system (system %u component %u)" % (
        self.master.target_system, self.master.target_component))
        self.command = 1

    def run(self):
        while not self.continue_flag.is_cancelled :
            match self.command:
                case 1: self.basic_nav()
                case 2:
                    self.land()
                case _: "waiting for command"

    def command(self, int_val):
        self.command = int_val

    def between_two_numbers(self, num, a, b):
        if b < a:
            a, b = b, a
        if a < num and num < b:
            return True
        else:
            return False

    def move_to_frd_ned(self, f, r, d):
        self.textLogger.log("move NED")
        self.master.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.master.target_system,
                                                                          self.master.target_component,
                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                          int(0b000011111000), f, r, d, 0, 0, 0,
                                                                          0, 0, 0, 0, 0))
        arrived = False
        while not arrived:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)  #
            self.textLogger.log(msg)
            if self.between_two_numbers(msg.x, f - .1, f + 0.1) and self.between_two_numbers(msg.y, r - .1,
                                                                                             r + 0.1) and self.between_two_numbers(
                msg.z,
                d - .1,
                d + 0.1):
                self.textLogger.log("arrived")
                arrived = True

    def rotate_cc_to(self, rotation_z, direction):
        self.textLogger.log("rotating")
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        start_yaw = math.degrees(msg.yaw)
        self.textLogger.log("start yaw {}".format(start_yaw))

        target_yaw = start_yaw + rotation_z
        if target_yaw > 180:
            target_yaw = target_yaw - 360

        self.textLogger.log("target yaw {}".format(target_yaw))

        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                          int(rotation_z), 5, direction, 1, 0, 0, 0, 0)

        arrived = False
        while not arrived:
            msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if msg is not None:
                current_yaw = math.degrees(msg.yaw)
                if current_yaw > 180:
                    current_yaw = current_yaw - 360
                if current_yaw < -180:
                    current_yaw = current_yaw + 360
                self.textLogger.log("target {} current {}".format(target_yaw, current_yaw))
                if self.between_two_numbers(current_yaw, target_yaw - 5, target_yaw + 5):
                    self.textLogger.log("arrived")
                    arrived = True

    def land(self):
        self.textLogger.log(">>>>>>>>>>START LANDING SEQUENCE<<<<<<<<<<<<<")
        self.cpt = 0
        arrived = False
        while not arrived:
            if self.landing_target_data_stack.qsize() > 0:
                try:
                    data = self.landing_target_data_stack.get()
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

                            if self.cpt == 10:
                                yaw = angles[2]
                                direction = 1
                                if yaw < 0:
                                    yaw = abs(yaw)
                                    direction = -1

                                self.master.mav.command_long_send(self.master.target_system,
                                                                  self.master.target_component,
                                                                  mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                                                  int(yaw), angle_rate_used, direction, 1, 0, 0, 0, 0)
                                self.cpt = 0
                            self.cpt += 1
                    else:
                        print("t_vec used is NOne")
                except queue.Empty:
                    print("queue is empty")

    def basic_nav(self):
        # move_to_frd_ned(0,0,-50)
        # Set all basic parameters for PRECISION LANDING TO work MAV_CMD_DO_SET_PARAMETER
        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                       b'RNGFND1_TYPE', 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                       b'LAND_SPEED', 50.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                       b'LAND_ALT_LOW', 400.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                       b'LAND_SPEED_HIGH', 500.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        self.textLogger.log("guided mode")

        # mode guide
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                          1, 4, 0, 0, 0, 0, 0, 0)
        self.textLogger.log("arm")

        # ARM
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                          1, 0, 0, 0, 0, 0, 0, 0)

        # takeoff 20
        self.textLogger.log("send takeoff")
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                          0, 0, 0, 0, 0, 0, 2)

        self.textLogger.log("Wait 20 sec")
        time.sleep(20)

        self.textLogger.log("travel to NED dest.")
        self.move_to_frd_ned(1, 2, -20)

        self.textLogger.log("rotate random degrees CCW")
        import random
        rot_rand = random.randint(0, 180)
        self.textLogger.log("ramdom value : {}".format(rot_rand))
        # self.rotate_cc_to(rot_rand, 1)

        # set land mode
        self.textLogger.log("Start landing")
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                          1, 9, 0, 0, 0, 0, 0, 0)

        self.command = 2