import time
import math
import numpy as np
import os
from video_from_drone import VideoCapture
from landingTargetDetector import LandingTargetDetector
from Navigator import Navigator
from textLogger import TextLogger
# import os
# os.environ['MAVLINK20']='1'

if __name__ == '__main__':
    from msgobj.cancellationToken import CancellationToken
    from landing_target import LandingTarget

    images_stack = []
    landing_target_data_stack = LandingTarget()
    analyzed_img_stack = []
    message_stack = []

    continue_flag = CancellationToken()

    textLogger = TextLogger(message_stack)
    capture = VideoCapture( images_stack)
    landingTargetDetector = LandingTargetDetector(images_stack, landing_target_data_stack, analyzed_img_stack)
    navigator = Navigator(landing_target_data_stack, textLogger)
    navigator.start()
    navigator.command = 1
