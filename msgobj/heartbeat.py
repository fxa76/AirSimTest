import threading
import time
from pymavlink import mavutil

class Heartbeat(threading.Thread):
    def __init__(self,theDrone):
        super(Heartbeat, self).__init__()
        self.master = theDrone.master
        self.distances = []
        for i in range(0,100):
            self.distances.append(100)

    def run(self):
        while True:
            time.sleep(1)
            print("heartbeat from computer")
            # self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            # print("hearbeat")
            self.master.mav.heartbeat_send(mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0,
                                    0,
                                    0)
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, "hello".encode())
            '''
            self.master.mav.obstacle_distance_send(
                int(round(time.time() * 1000000)),  # us Timestamp (UNIX time or time since system boot)
                0,  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
                self.distances,  # distances,    uint16_t[72],   cm
                0,  # increment,    uint8_t,        deg
                100,  # min_distance, uint16_t,       cm
                100,  # max_distance, uint16_t,       cm
                50.10,  # increment_f,  float,          deg
                0.1,# angle_offset, float,          deg
                12
                # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD
            )
            
            self.master.mav.command_long_send(
                255, 0,
                253,0,#mavutil.mavlink.MAVLink_statustext_message, 0,
                mavutil.mavlink.MAV_SEVERITY_ALERT,  #severity
                b'hello', # text
                0, # id
                0, # chunk id
                0, #chunk seq
                0,
                0
            )
            msg = self.gcs.recv_match(blocking=False)
            if msg is not None:
                msg_dict = msg.to_dict()
                print(msg_dict)
            '''

