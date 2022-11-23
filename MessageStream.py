import time
# Import mavutil
from pymavlink import mavutil

import threading
import numpy as np

class MessageStream(threading.Thread):
    def __init__(self):
        super(MessageStream, self).__init__()
        # Create the connection
        self.master = mavutil.mavlink_connection(
            "udp:192.168.1.30:14560")  # in Arducopter cmd type: output add 192.168.1.18:14560
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

        from msgobj.Attitude import ATTITUDE
        from msgobj.PositionNED import PositionNED

        self.attitude = ATTITUDE()
        self.positionNed = PositionNED()

    def request_message_interval(self,message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def run(self):
        # Configure AHRS2 message to be sent at 1Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)

        # Configure ATTITUDE message to be sent at 50Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)

        # Configure ATTITUDE message to be sent at 50Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)


        # Get some information !
        while True:
            try:
                msg_dict = self.master.recv_match().to_dict()
                if msg_dict['mavpackettype'] == 'ATTITUDE':
                    self.attitude.update(msg_dict)
                else:
                    if msg_dict['mavpackettype'] == 'LOCAL_POSITION_NED':
                        self.positionNed.update(msg_dict)
                    else:
                        print(msg_dict)
                        
            except:
                print("error")
                pass
            time.sleep(0.01)

if __name__ == '__main__':
    msgStream = MessageStream()
    msgStream.start()