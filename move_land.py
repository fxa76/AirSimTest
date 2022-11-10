import time
import math
import numpy as np

from video_from_drone import VideoCapture, FileWriter
from scipy.spatial.transform import Rotation

#import os

#os.environ['MAVLINK20']='1'

from pymavlink import mavutil

print(">>CONNECTING TO DRONE<<")

master = mavutil.mavlink_connection("udp:192.168.1.30:14560") #in Arducopter cmd type: output add 192.168.1.18:14560

master.mav.ping_send(
    int(time.time() * 1e6), # Unix time in microseconds
    0, # Ping number
    0, # Request ping of all systems
    0 # Request ping of all components
)
print(">>WAITING FOR HEARTBEAT<<")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

def rotate(angle_x, angle_y):
    """
    Actual calcuation:
        sin_yaw = math.sin(rotation/180*math.pi)
        #cos_yaw = math.cos(rotation/180*math.pi)
        cos_yaw = 0 # Python math doesn't actually make it zero though
        rot = np.array([[cos_yaw, -sin_yaw, 0],[sin_yaw,cos_yaw,0],[0,0,1]])
        vec = np.array([1,2,3]) # y, x, z
        rot.dot(vec) # returns: array([ 2., -1.,  3.]) # y, x, z = x, -y, z
        i.e. we get: x, y = -y, x
    Reference frame of ArduCopter -- x forward, y right
    https://discuss.ardupilot.org/t/copter-x-y-z-which-is-which/6823/3
    https://docs.px4.io/en/config/flight_controller_orientation.html
    Basically, try it till it works. My RPi is mounted so the top of the image
    is actually the left side of my drone and the right of the image is the front.
    Actually, I think that +y in the image is the bottom of the image. Thus,
    by mounting the RPi in that way, I think it's actually outputting (x,y)
    coordinates in the same frame as the drone.
    See: https://github.com/tensorflow/tensorflow/issues/9142
    Or.... from this Youtube comment
    https://www.youtube.com/watch?v=5AVb2hA2EUs&lc=UggMS88TyPsEt3gCoAEC.8I2Q1bOkhXh8I3ZMGRRKTY
    saying "negative X = target to the left, negative Y = target is forward", then
    we want: angle_y, -angle_x
    """
    return angle_y, -angle_x # from Youtube comment and TF origin in top left

def between_two_numbers(num,a,b):
    if b < a:
        a, b = b, a
    if a<num and num<b:
        return True
    else:
        return False


def move_to_frd_ned(f,r,d):
    print("move NED")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,master.target_system,
                                                                      master.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                              int(0b000011111000),f,r,d,0,0,0,0,0,0,0,0))
    arrived = False
    while not arrived:
        msg = master.recv_match(type='LOCAL_POSITION_NED',blocking=True) #
        print(msg)
        if between_two_numbers(msg.x,f-.1,f+0.1) and between_two_numbers(msg.y,r-.1,r+0.1) and between_two_numbers(msg.z,d-.1,d+0.1):
            print("arrived")
            arrived = True

def land():
    print(">>>>>>>>>>LANDING<<<<<<<<<<<<<")

    arrived = False
    while not arrived:
        if len(landing_target)>0:
            #print(len(landing_target))
            data = landing_target.pop()
            rvec = data[0]
            tvec = data[1]
            t_vec_big = data[2]
            t_vec_small = data[3]
            r_vec_big = data[4]
            t_vec_used = None
            using= "None"
            if t_vec_big is not None:
                t_vec_used = t_vec_big
                using = "Big"
            if t_vec_small is not None:
                t_vec_used = t_vec_small
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

                r_vec_big = np.asarray(r_vec_big)
                r= Rotation.from_matrix(r_vec_big)
                angles = r.as_euler("xyz",degrees=True)
                quat = r.as_quat()
                print("rot {}".format(angles))
                print("quat {}".format(quat))
                quat[0]=quat[0]*100
                quat[1] = quat[1] * 100
                quat[2] = quat[2] * 100
                quat[3] = quat[3] * 100
                print("quat {}".format(quat))
                #print("{}: {},{},{}".format(using, forward, right, height))
                master.mav.landing_target_send(
                                        0, #time_usec
                                        1, #target_num
                                        mavutil.mavlink.MAV_FRAME_BODY_FRD, # MAV Frame
                                        0, #angle_x
                                        0, #angle_y
                                        height,#distance height
                                        0.5, #size_x
                                        0.5, #size_y
                                        forward, # x** forward
                                        right, #y** right
                                        height, # z** height
                                        quat, #q** no rotation [1,0,0,0]
                                        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                                        1)
                yaw = angles[2]
                direction = 1
                if yaw < 0:
                    yaw = abs(yaw)
                    direction= -1

                master.mav.command_long_send(master.target_system, master.target_component,
                                             mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                                             int(yaw), 5, direction, 1, 0, 0, 0, 0)


print("starting video")
stack = []
landing_target = []
capture = VideoCapture(stack)
fileWriter = FileWriter(stack,landing_target)

#move_to_frd_ned(0,0,-50)
land()