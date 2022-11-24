from msgobj.ThreadSafeSingleton import SingletonMeta
from msgobj.MessageStream import MessageStream
from msgobj.StatusText import StatusText

from pymavlink import mavutil

class Drone(metaclass=SingletonMeta):

    def __init__(self) -> None:

        # Create the connection
        self.master = mavutil.mavlink_connection(
           "udp:192.168.1.30:14560")  # in Arducopter cmd type: output add 192.168.1.18:14560
        # Wait a heartbeat before sending commands
        print("Waiting for heart beat")
        self.master.wait_heartbeat()
        print("got it")
        from msgobj.Attitude import ATTITUDE
        from msgobj.PositionNED import PositionNED

        self.attitude = ATTITUDE()
        self.positionNed = PositionNED()
        self.statustext = StatusText()

        self.messageStream = MessageStream(self)
        self.messageStream.start()

    def __repr__(self):
        return "Position Ned {} + Attitude {}".format(self.positionNed,self.attitude)

if __name__ == '__main__':
    drone = Drone("udp:192.168.1.30:14560")
