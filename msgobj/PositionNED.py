class PositionNED():
    def __init__(self):
        self.time_boot_ms = None
        self.x = None
        self.y = None
        self.z = None
        self.vx = None
        self.vy = None
        self.vz = None


    def update(self,mavpackeftype_LOCAL_POSITION_NED : dict):
        self.time_boot_ms = mavpackeftype_LOCAL_POSITION_NED['time_boot_ms']
        self.x = mavpackeftype_LOCAL_POSITION_NED['x']
        self.y = mavpackeftype_LOCAL_POSITION_NED['y']
        self.z = mavpackeftype_LOCAL_POSITION_NED['z']
        self.vx = mavpackeftype_LOCAL_POSITION_NED['vx']
        self.vy = mavpackeftype_LOCAL_POSITION_NED['vy']
        self.vz = mavpackeftype_LOCAL_POSITION_NED['vz']
        #print("Position ned updated")

    def __repr__(self):
        return "x: {}, y: {}, z:{}".format(self.x,self.y,self.z)