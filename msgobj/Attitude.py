class Attitude:
    def __init__(self):
        self.time_boot_ms = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.rollspeed = None
        self.pitchspeed = None
        self.yawspeed = None

    def update(self,mavpackeftype_ATTITUDE : dict):
        self.time_boot_ms = mavpackeftype_ATTITUDE['time_boot_ms']
        self.roll = mavpackeftype_ATTITUDE['roll']
        self.pitch = mavpackeftype_ATTITUDE['pitch']
        self.yaw = mavpackeftype_ATTITUDE['yaw']
        self.rollspeed = mavpackeftype_ATTITUDE['rollspeed']
        self.pitchspeed = mavpackeftype_ATTITUDE['pitchspeed']
        self.yawspeed = mavpackeftype_ATTITUDE['yawspeed']
        #print("Attitude updated")