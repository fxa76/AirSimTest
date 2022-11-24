from PyQt5.QtWidgets import QApplication
from views import StartWindow
from move_land import Navigator, TextLogger
from video_from_drone import VideoCapture
from landingTargetDetector import LandingTargetDetector
from msgobj.cancellationToken import CancellationToken

if __name__ == '__main__':
    from landing_target import LandingTarget

    source_images_stack = []
    landing_target_data = LandingTarget()
    analyzed_img_stack = []
    message_stack = []
    continue_flag = CancellationToken()

    textLogger = TextLogger(message_stack)
    capture = VideoCapture(source_images_stack)
    landingTargetDetector = LandingTargetDetector(source_images_stack, landing_target_data, analyzed_img_stack)
    navigator = Navigator(landing_target_data,textLogger)
    navigator.start()

    app = QApplication([])
    start_window = StartWindow(analyzed_img_stack,message_stack,navigator,landing_target_data)
    start_window.show()
    app.exit(app.exec_())
    continue_flag.cancel()