import time
# Import mavutil
from pymavlink import mavutil
import pymavlink
import threading
import numpy as np

from msgobj.cancellationToken import CancellationToken


class MessageStream(threading.Thread):
    def __init__(self,theDrone):
        super(MessageStream, self).__init__()
        self.drone = theDrone
        self.master = self.drone.master
        #self.cancelToken = CancellationToken()


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
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 4)

        # Configure ATTITUDE message to be sent at 50Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 4)


        # Get some information !
        while True:#self.cancelToken:
            time.sleep(0.01)
            try:
                msg = self.master.recv_match(blocking=True)
                if msg is not None:
                    msg_dict = msg.to_dict()
                    if msg_dict['mavpackettype'] == 'ATTITUDE':
                        self.drone.attitude.update(msg_dict)

                    if msg_dict['mavpackettype'] == 'LOCAL_POSITION_NED':
                        self.drone.positionNed.update(msg_dict)
                        #print(msg_dict['z'])
                    if msg_dict['mavpackettype'] == 'STATUSTEXT':
                        self.drone.statustext.update(msg_dict)
                        #print(msg_dict['z'])
                    else:
                        #print(msg_dict)
                        pass

            except Exception as err:
                print("Error {}".format(err))
                pass
