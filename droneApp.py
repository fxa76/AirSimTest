from PyQt5.QtWidgets import QApplication
from views import StartWindow
from move_land import Navigator, TextLogger
from video_from_drone import VideoCapture, LandingTargetDetector


if __name__ == '__main__':

    images_stack = []
    landing_target_data_stack = []
    analyzed_img_stack = []
    message_stack = []
    continue_flag = True
    textLogger = TextLogger(message_stack)
    capture = VideoCapture(continue_flag, images_stack)
    landingTargetDetector = LandingTargetDetector(continue_flag, images_stack, landing_target_data_stack,
                                                  analyzed_img_stack)
    navigator = Navigator(landing_target_data_stack,textLogger)

    app = QApplication([])
    start_window = StartWindow(analyzed_img_stack,message_stack,navigator)
    start_window.show()
    app.exit(app.exec_())