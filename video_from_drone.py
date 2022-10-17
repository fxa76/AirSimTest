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
        thread = threading.Thread(target=self.start, )
        thread.start()

    def __del__(self):
        print("calling destructor")

    def start(self):
        seq = 0
        while True:

            if len(self.stack) > 0 :
                # print("stck length{}".format(len(self.stack)))
                response_image = self.stack.pop()
                np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
                decoded_frame = cv2.imdecode(np_response_image, cv2.IMREAD_COLOR)

                if decoded_frame is not None:
                    # print("display")
                    cv2.imshow("Drone camera",decoded_frame)
                    if cv2.waitKey(1) == ord("q"):
                        cv2.destroyAllWindows()
                        exit(0)


class VideoCapture:

    def __init__(self,stack ):
        self.stack = stack
        simclient = airsim.MultirotorClient()
        simclient.confirmConnection()
        self.client = simclient
        # print(self.client.simGetCameraInfo(str('down')))
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


if __name__ == '__main__':
    stack = []
    capture = VideoCapture(stack)
    fileWriter = FileWriter(stack)
