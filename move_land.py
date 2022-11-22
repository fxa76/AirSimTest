import time
import math
import numpy as np
import os
from video_from_drone import VideoCapture
from landingTargetDetector import LandingTargetDetector
from scipy.spatial.transform import Rotation
from Navigator import Navigator

# import os
# os.environ['MAVLINK20']='1'

from pymavlink import mavutil
import threading

class TextLogger:
    def __init__(self, message_stack):
        self.message_stack = message_stack

    def log(self, text):

        self.message_stack.append(text)
        print("{} : {}".format(len( self.message_stack), text))



if __name__ == '__main__':
    from cancellationToken import CancellationToken
    from landing_target import LandingTarget

    images_stack = []
    landing_target_data_stack = LandingTarget()
    analyzed_img_stack = []
    message_stack = []

    continue_flag = CancellationToken()

    textLogger = TextLogger(message_stack)
    capture = VideoCapture(continue_flag, images_stack)
    landingTargetDetector = LandingTargetDetector(continue_flag,images_stack, landing_target_data_stack, analyzed_img_stack)
    navigator = Navigator(continue_flag,landing_target_data_stack, textLogger)
    navigator.start()
    navigator.command = 1
