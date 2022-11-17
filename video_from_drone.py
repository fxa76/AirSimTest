import threading
import time
from pupil_apriltags import Detector
import cv2
import numpy as np
import time
import imutils
import airsim
from findHomographyORB_GPU import find_homography
from arucode_decoder import Arucode_Decoder

class LandingTargetDetector():
    def __init__(self,continue_flag, stack,targetStack,analyzed_img_stack):
        self.continue_flag = continue_flag
        self.stack = stack
        self.targetStack = targetStack
        self.analyzed_img_stack = analyzed_img_stack

        #self.decoder = Arucode_Decoder()
        self.at_detector = Detector(
            families="tagCustom48h12",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.at_detector2 = Detector(
            families="tagCircle21h7",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        thread = threading.Thread(target=self.start, )
        thread.start()

    def __del__(self):
        print("calling destructor")

    def draw_contour(self,tag,decoded_frame):
        # print(tag_big)
        (topLeft, topRight, bottomRight, bottomLeft) = tag.corners
        markerID = "{},{}".format(tag.tag_family, tag.tag_id)
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # draw the bounding box of the ArUCo detection
        cv2.line(decoded_frame, topLeft, topRight, (255, 0, 0), 2)
        cv2.line(decoded_frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(decoded_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(decoded_frame, bottomLeft, topLeft, (0, 255, 0), 2)

        cv2.putText(decoded_frame, "{}: {} ".format(str(markerID), "Big Marker Landing Pad"),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        return decoded_frame

    def start(self):
        seq = 0
        while self.continue_flag:

            if len(self.stack) > 0 :
                # print("stck length{}".format(len(self.stack)))
                response_image = self.stack.pop()
                np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
                decoded_frame = cv2.imdecode(np_response_image, cv2.IMREAD_COLOR)
                if decoded_frame is not None:
                    decoded_frame_gray =  cv2.cvtColor(decoded_frame, cv2.COLOR_BGR2GRAY)

                    # decode arucode
                    #decoded_frame, rvec, tvec = self.decoder.decode(decoded_frame)
                    #decode april tags
                    tag_big = self.at_detector.detect(decoded_frame_gray, camera_params=[90, 90, 512, 768/2], estimate_tag_pose=True, tag_size=.5)
                    tag_small = self.at_detector2.detect(decoded_frame_gray, camera_params=[90, 90, 512, 768/2], estimate_tag_pose=True, tag_size=.1)

                    t_vec_big = None
                    r_vec_big = None
                    t_vec_small = None
                    r_vec_small = None

                    if len(tag_big)>0:
                        t_vec_big =tag_big[0].pose_t
                        r_vec_big = tag_big[0].pose_R
                        decoded_frame = self.draw_contour(tag_big[0],decoded_frame)
                    if len(tag_small)>0:
                        t_vec_small =tag_small[0].pose_t
                        r_vec_small = tag_small[0].pose_R
                        decoded_frame = self.draw_contour(tag_small[0],decoded_frame)

                    self.targetStack.append([t_vec_big,t_vec_small,r_vec_big,r_vec_small])

                    if (decoded_frame is None):
                        print("image is none")
                    else:
                        self.analyzed_img_stack.append(decoded_frame)
                        '''
                        img = imutils.resize(decoded_frame, width=600)
                        cv2.imshow("Drone camera",img)
                        if cv2.waitKey(1) == ord("q"):
                            cv2.destroyAllWindows()
                            exit(0)
                        '''


class VideoCapture:

    def __init__(self,continue_flag,stack ):
        self.continue_flag = continue_flag
        self.stack = stack
        simclient = airsim.MultirotorClient()
        simclient.confirmConnection()
        self.client = simclient
        # print(self.client.simGetCameraInfo(str('down')))
        thread = threading.Thread(target=self.start, )
        thread.start()

    def __del__(self):
        print("calling destructor")

    def start(self):
        CAMERA_NAME = 'down'  # 'high_res'
        IMAGE_TYPE = airsim.ImageType.Scene

        while self.continue_flag:
            response_image = self.client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
            self.stack.append(response_image)
            #time.sleep(0.2)


if __name__ == '__main__':
    image_stack = []
    data_stack = []
    capture = VideoCapture(image_stack)
    detector = LandingTargetDetector(image_stack,data_stack)
