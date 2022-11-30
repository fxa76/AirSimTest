from PyQt5.QtWidgets import QApplication
from views import StartWindow
from move_land import Navigator, TextLogger
from video_from_drone import VideoCaptureAirsim
from video_from_usb_camera import VideoCaptureUSB
from landingTargetDetector import LandingTargetDetector
from msgobj.cancellationToken import CancellationToken
from landing_target import LandingTarget


if __name__ == '__main__':

    source_images_stack = []
    landing_target_data = LandingTarget()
    analyzed_img_stack = []
    message_stack = []
    continue_flag = CancellationToken()

    textLogger = TextLogger(message_stack)
    capture = VideoCaptureAirsim(source_images_stack)
    landingTargetDetector = LandingTargetDetector(source_images_stack, landing_target_data, analyzed_img_stack)
    navigator = Navigator(landing_target_data, textLogger)
    navigator.start()

    app = QApplication([])
    start_window = StartWindow(analyzed_img_stack, message_stack, navigator, landing_target_data)
    start_window.show()
    app.exit(app.exec_())
    continue_flag.cancel()