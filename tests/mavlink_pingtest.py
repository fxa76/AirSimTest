import time
#import os

#os.environ['MAVLINK20']='1'

from pymavlink import mavutil


def cmd_takeoff(master,target, altitude):
    '''take off'''
    print("Take Off started")
    master.mav.command_long_send(
        self.settings.target_system,  # target_system
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
print(">>GETTING LOCATION")

print(">>ARMING !!!<<")
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0,0)
msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)
while True:
    try:
        msg = master.recv_match(blocking=True)
        print(msg)
    except:
        print("exception")
    time.sleep(0.1)