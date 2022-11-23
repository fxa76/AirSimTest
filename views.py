from PyQt5.QtGui import QImage, QPixmap, QPainter
from PyQt5.QtCore import Qt, QThread, QTimer
from PyQt5.QtWidgets import QMainWindow, QFormLayout, QWidget, QPushButton, QBoxLayout, QVBoxLayout, QApplication, \
    QSlider, QLabel, QGridLayout, QGroupBox, QCheckBox, QSizePolicy, QTextEdit

import cv2
import os
import datetime
import threading


class StartWindow(QMainWindow):
    def __init__(self,analyzed_img_stack,message_stack,navigator):
        super().__init__()

        self.analyzed_img_stack = analyzed_img_stack
        self.message_stack = message_stack
        self.navigator = navigator

        self.central_widget = QWidget()

        self.button_takeoff = QPushButton('takeoff', self.central_widget)
        self.button_land = QPushButton('land', self.central_widget)
        self.button_land_done = QPushButton('landing done', self.central_widget)

        self.image_view = Label()
        self.status_text = QTextEdit("")

        self.left = 100
        self.top = 100
        self.width = 320
        self.height = 375
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.horizontalGroupBox = QGroupBox("Live Capture")
        #self.horizontalGroupBox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout2 = QGridLayout()
        layout2.addWidget(self.image_view, 0, 0)
        layout2.addWidget(self.status_text,0, 1)
        layout2.setColumnStretch(0, 1)
        layout2.setColumnStretch(1, 1)

        self.horizontalGroupBox.setLayout(layout2)

        self.layout = QVBoxLayout(self.central_widget)
        self.layout.addWidget(self.button_takeoff)
        self.layout.addWidget(self.button_land)
        self.layout.addWidget(self.button_land_done)
        self.layout.addWidget(self.horizontalGroupBox)

        self.setCentralWidget(self.central_widget)

        # set button and sliders actions
        self.button_takeoff.clicked.connect(self.takeoff_call_back)
        self.button_land.clicked.connect(self.land_call_back)
        self.button_land_done.clicked.connect(self.land_done_call_back)


        self.video_thread = VideoUpdateThread(self.analyzed_img_stack, self)
        self.video_thread.start(1)

        self.msg_thread = MessageUpdateThread(self.message_stack, self)
        self.msg_thread.start(1)

    def takeoff_call_back(self):
        print("takeoff_call_back")
        print("go on ")
        self.navigator.command=1

    def land_call_back(self):
        print("land_call_back")
        self.navigator.command = 2

    def land_done_call_back(self):
        print("land_done_call_back")
        self.navigator.landed = True
        self.navigator.command = None

    def convert_to_qpixmap_color(self, cv_img):
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, byte_value = cv_img.shape
        # print("{} and {} and {}".format(height,width,byte_value))
        byte_value = byte_value * width
        qimage = QImage(cv_img, width, height, byte_value, QImage.Format_RGB888)
        return qimage

    def convert_to_qpixmap(self, cv_img):
        height, width, byte_value = cv_img.shape
        byte_value = byte_value * width
        qimage = QImage(cv_img, width, height, byte_value, QImage.Format_RGB888)
        return qimage


class Label(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.p = QPixmap()

    def setPixmap(self, p):
        self.p = p
        self.update()

    def paintEvent(self, event):
        if not self.p.isNull():
            painter = QPainter(self)
            painter.setRenderHint(QPainter.SmoothPixmapTransform)
            painter.drawPixmap(self.rect(), self.p)


class VideoUpdateThread(QThread):
    def __init__(self, analyzed_img_stack, window):
        super().__init__()
        self.analyzed_img_stack = analyzed_img_stack
        self.window = window


    def run(self):
        while True:
            # print("updating video")
            if len(self.analyzed_img_stack) > 0:
                qimage = self.window.convert_to_qpixmap_color(self.analyzed_img_stack.pop())
                self.window.image_view.setPixmap(QPixmap.fromImage(qimage))

class MessageUpdateThread(QThread):
    def __init__(self, message_stack,window):
        super().__init__()
        self.message_stack = message_stack
        self.window = window

    def run(self):
        while True:
            # print("status text update")
            if len(self.message_stack) > 0:
                msg = self.message_stack.pop()
                self.window.status_text.append(msg)

if __name__ == '__main__':
    app = QApplication([])
    window = StartWindow()
    window.show()
    app.exit(app.exec_())