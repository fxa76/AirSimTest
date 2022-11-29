import threading
import time

import cv2
import numpy as np
import time

import airsim
from findHomographyORB_GPU import find_homography

class FileWriter():
    def __init__(self,stack):
        self.stack = stack
        self.start()
        pass

    def __del__(self):
        print("calling destructor")

    def start(self):
        seq = 0
        while True:

            if len(self.stack) > 0 :
                print("stck length{}".format(len(self.stack)))
                response_image = self.stack.pop()
                np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
                decoded_frame = cv2.imdecode(np_response_image, cv2.IMREAD_COLOR)

                if decoded_frame is not None:
                    cv2.imwrite('./movie/{}.png'.format(str(seq).zfill(5)), decoded_frame)
                    seq = seq + 1


class VideoCapture:

    def __init__(self,stack ):
        self.stack = stack
        simclient = airsim.MultirotorClient()
        simclient.confirmConnection()
        self.client = simclient
        print(self.client.simGetCameraInfo(str('down')))
        thread = threading.Thread(target=self.start, )
        self.continue_flag = True
        thread.start()

    def __del__(self):
        print("calling destructor")

    def start(self):
        CAMERA_NAME = 'down'  # 'high_res'
        IMAGE_TYPE = airsim.ImageType.Scene
        DECODE_EXTENSION = '.jpg'
        # img1 = cv2.imread("target2.png")


        while self.continue_flag:
            response_image = self.client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
            self.stack.append(response_image)
            #time.sleep(0.2)

def flyandrecord():
    z = 5

    client = airsim.MultirotorClient()

    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    capture = VideoCapture()

    landed = client.getMultirotorState().landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
    else:
        print("already flying...")
        client.hoverAsync().join()

    client.moveToZAsync(-120, 5).join()
    print("make sure we are hovering at {} meters...".format(z))
    client.moveToPositionAsync(40, 40, -120, 5).join()

    client.moveToPositionAsync(0, 0, -80, 5).join()

    # if z > 5:
    #     # AirSim uses NED coordinates so negative axis is up.
    #     # z of -50 is 50 meters above the original launch point.
    #     client.moveToZAsync(-z, 5).join()
    #     client.hoverAsync().join()
    #     time.sleep(5)
    #
    # if z > 10:
    #     print("come down quickly to 10 meters...")
    #     z = 10
    #     client.moveToZAsync(-z, 3).join()
    #     client.hoverAsync().join()

    print("landing...")
    client.landAsync().join()
    print("disarming...")
    client.armDisarm(False)
    capture.continue_flag = False
    client.enableApiControl(False)
    print("done.")

if __name__ == '__main__':
    stack = []
    capture = VideoCapture(stack)
    fileWriter = FileWriter(stack)
