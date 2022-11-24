import airsim
import threading
import numpy as np
import cv2
from msgobj.cancellationToken import CancellationToken


class VideoCapture:

    def __init__(self,stack ):
        self.continue_flag = CancellationToken()
        self.stack = stack
        simclient = airsim.MultirotorClient()
        simclient.confirmConnection()
        self.client = simclient
        # print(self.client.simGetCameraInfo(str('down')))
        thread = threading.Thread(target=self.start, )
        thread.start()

    def __del__(self):
        print("video from drone stopped")

    def start(self):
        CAMERA_NAME = 'down'  # 'high_res'
        IMAGE_TYPE = airsim.ImageType.Scene

        while not self.continue_flag.is_cancelled:
            response_image = self.client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
            np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
            decoded_frame = cv2.imdecode(np_response_image, cv2.IMREAD_COLOR)
            if decoded_frame is not None:
                self.stack.append(decoded_frame)
                #time.sleep(0.2)


if __name__ == '__main__':
    import time

    source_image_stack = []
    continue_flag = CancellationToken()
    capture = VideoCapture(source_image_stack)
    time_len= 10
    time.sleep(time_len)
    print("number of images stored after {} sec: {}".format(time_len, len(source_image_stack)))
    continue_flag.cancel()
    print("continue set to False")
