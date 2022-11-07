import time
import math

from video_from_drone import VideoCapture, FileWriter

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
            print(data)
            x = data[0]
            y = data[1]
            corners = data[2]
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            width = abs(topRight[1] - topLeft[1])
            print(width)

            # https://github.com/floft/vision-landing/blob/master/autopilot_communication_processes.py
            horizontal_fov = 90
            vertical_fov = 90
            horizontal_fov = horizontal_fov / 180 * math.pi
            vertical_fov = vertical_fov / 180 * math.pi
            horizontal_resolution = 1024
            vertical_resolution = 768
            angle_x = (x - horizontal_resolution / 2) * horizontal_fov / horizontal_resolution
            angle_y = (y - vertical_resolution / 2) * vertical_fov / vertical_resolution
            # angle_x, angle_y = rotate(angle_x, angle_y)
            target_size = 50
            size_x = width  * horizontal_fov / horizontal_resolution
            larger_size = size_x

            height = (target_size / 2) / math.tan(larger_size / 2)
            print("angle x {}, angle y {}, height {}".format(angle_x,angle_y,height))
            '''
            master.mav.landing_target_send(
                mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame (not used)
                10,
                10,
                10,
                0,
                2,
                0,
                0)
            '''
            master.mav.landing_target_send(
                                        0, #time_usec
                                        1, #target_num
                                        mavutil.mavlink.MAV_FRAME_BODY_NED, # MAV Frame
                                        angle_x, #angle_x
                                        angle_y, #angle_y
                                        height,#distance
                                        0.1, #size_x
                                        0.1, #size_y
                                        0, #x**
                                        0, # y**
                                        0, # z**
                                        [1,0,0,0], #q**
                                        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                                        0)


print("starting video")
stack = []
landing_target = []
capture = VideoCapture(stack)
fileWriter = FileWriter(stack,landing_target)

move_to_frd_ned(0,0,-25)
land()


'''

master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10,master.target_system,
                                                                      master.target_component,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                              int(0b11011111000),int(-35.3629849 * 10 **7),int(149.1649185 * 10 **7),10,0,0,0,0,0,0,0,0))
'''
