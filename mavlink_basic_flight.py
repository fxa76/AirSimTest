import time
from video_from_drone import VideoCapture, FileWriter
#import os

#os.environ['MAVLINK20']='1'

from pymavlink import mavutil
import pymavlink

def cmd_move_relative_to_drone(master,target_system,f,r,d):
    '''move to frd'''
    print("frd started")
    master.mav.command_long_send(
        target_system,  # target_system
        mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL,  # target_component
        mavutil.mavlink.COPTER_MODE_RTL  ,  # command
        0,  # confirmation
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        0)  # param7
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)


def cmd_takeoff(master,target_system, altitude):
    '''take off'''
    print("Take Off started")
    master.mav.command_long_send(
        target_system,  # target_system
        mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,  # confirmation
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        altitude)  # param7
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

print(">>CONNECTING TO DRONE<<")

master = mavutil.mavlink_connection("udp:192.168.1.18:14560") #in Arducopter cmd type: output add 192.168.1.18:14560

print("starting video")
stack = []
capture = VideoCapture(stack)
fileWriter = FileWriter(stack)

master.mav.ping_send(
    int(time.time() * 1e6), # Unix time in microseconds
    0, # Ping number
    0, # Request ping of all systems
    0 # Request ping of all components
)
print(">>WAITING FOR HEARTBEAT<<")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

print(">>MAV_CMD_DO_SET_HOME<<")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME,0,0,0,0,0,0,0,0,0)
msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

print(">>ARMING !!!<<")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0,0)
msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

print(">>Take off<<")
target_height=5
cmd_takeoff(master,master.target_system, target_height)

msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
print(msg.relative_alt)
print(msg)

continue_flag = True
lat=0.0
lon=0.0
while continue_flag:
     try:
         msg = master.recv_match(type= "GLOBAL_POSITION_INT", blocking=True)
         print(msg.relative_alt)
         print(msg)
         if msg.relative_alt > target_height*1000*.95:
             continue_flag=False
             lat = msg.lat
             lon = msg.lon

     except:
         print("exception")

print("move NED")
master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,master.target_system,
                                                                      master.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,int(0b110111111000),10,0,-10,0,0,0,0,0,0,0,0))
while 1:
    msg = master.recv_match(type='MAV_CONTROLLER_OUTPUT',blocking=True)
    print(msg)

print(">>MAV_CMD_NAV_RETURN_TO_LAUNCH")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0)
msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)
