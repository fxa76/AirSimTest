import airsim
import threading
import cv2
from msgobj.cancellationToken import CancellationToken


class VideoCaptureUSB:

    def __init__(self,stack ):
        self.continue_flag = CancellationToken()
        self.stack = stack
        print("waiting for camera")
        self.cap = cv2.VideoCapture(0)
        print("camera ready")
        # print(self.client.simGetCameraInfo(str('down')))
        thread = threading.Thread(target=self.start, )
        thread.start()

    def __del__(self):
        print("video from drone stopped")
        self.cap.release()

    def start(self):
        CAMERA_NAME = 'down'  # 'high_res'
        IMAGE_TYPE = airsim.ImageType.Scene

        while not self.continue_flag.is_cancelled:
            time.sleep(0.01)
            ret, frame = self.cap.read()
            if frame is not None:
                self.stack.append(frame)



if __name__ == '__main__':
    import time

    source_image_stack = []
    continue_flag = CancellationToken()
    capture = VideoCaptureUSB(source_image_stack)
    time_len= 10
    time.sleep(time_len)
    print("number of images stored after {} sec: {}".format(time_len, len(source_image_stack)))
    continue_flag.cancel()
    print("continue set to False")
