from pyqtgraph.Qt import QtCore


class SoftwareCheckThread(QtCore.QThread):

    def __init__(self, check):
        QtCore.QThread.__init__(self)
        self.check = check

    def run(self):
        self.check.perform()
