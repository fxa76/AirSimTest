from msgobj.ThreadSafeSingleton import SingletonMeta
from msgobj.MessageStream import MessageStream
from msgobj.StatusText import StatusText
from msgobj.heartbeat import Heartbeat

from pymavlink import mavutil

class Drone(metaclass=SingletonMeta):

    def __init__(self) -> None:

        # Create the connection
        self.master = mavutil.mavlink_connection(
           "udp:192.168.1.30:14560",notimestamps=True, source_system=1, source_component=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, autoreconnect=True, force_connected=True,)  # in Arducopter cmd type: output add 192.168.1.18:14560
        # Wait a heartbeat before sending commands
        print("Waiting for heart beat from drone")
        self.master.wait_heartbeat()
        print("got it")
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))

        from msgobj.Attitude import Attitude
        from msgobj.PositionNED import PositionNED

        self.attitude = Attitude()
        self.positionNed = PositionNED()
        self.statustext = StatusText()

        self.messageStream = MessageStream(self)
        self.messageStream.start()

        self.heartbeat = Heartbeat(self)
        self.heartbeat.start()

    def __repr__(self):
        return "Position Ned {} + Attitude {}".format(self.positionNed,self.attitude)

if __name__ == '__main__':
    drone = Drone()
