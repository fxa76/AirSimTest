import time
from uncertainties import ufloat

from video_from_drone import VideoCapture, FileWriter
#import os

#os.environ['MAVLINK20']='1'

from pymavlink import mavutil

print(">>CONNECTING TO DRONE<<")

master = mavutil.mavlink_connection("udp:192.168.1.18:14560") #in Arducopter cmd type: output add 192.168.1.18:14560

master.mav.ping_send(
    int(time.time() * 1e6), # Unix time in microseconds
    0, # Ping number
    0, # Request ping of all systems
    0 # Request ping of all components
)
print(">>WAITING FOR HEARTBEAT<<")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

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

move_to_frd_ned(0,0,-20)
move_to_frd_ned(10,10,-10)
move_to_frd_ned(10,-10,-10)
move_to_frd_ned(-10,-10,-10)
move_to_frd_ned(-10,10,-10)
move_to_frd_ned(0,0,-10)

'''

master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10,master.target_system,
                                                                      master.target_component,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                              int(0b11011111000),int(-35.3629849 * 10 **7),int(149.1649185 * 10 **7),10,0,0,0,0,0,0,0,0))
'''
