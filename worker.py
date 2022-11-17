# worker.py
# from article https://stackoverflow.com/questions/6783194/background-thread-with-qthread-in-pyqt
from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot
import time


class Worker(QObject):
    finished = pyqtSignal()
    intReady = pyqtSignal(int)


    @pyqtSlot()
    def procCounter(self): # A slot takes no params
        for i in range(1, 10):
            time.sleep(1)
            self.intReady.emit(i)

        self.finished.emit()