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
            rvec = data[0]
            tvec = data[1]
            if tvec is None:
                pass
            else:
                print("{},{},{}".format(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]))
                master.mav.landing_target_send(
                                        0, #time_usec
                                        1, #target_num
                                        mavutil.mavlink.MAV_FRAME_BODY_FRD, # MAV Frame
                                        0, #angle_x
                                        0, #angle_y
                                        tvec[0][0][2],#distance height
                                        0.1, #size_x
                                        0.1, #size_y
                                        0.25-tvec[0][0][1], # x** forward
                                        tvec[0][0][0]+0.25, #y** right
                                        tvec[0][0][2], # z** height
                                        [1,0,0,0], #q**
                                        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                                        1)


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
