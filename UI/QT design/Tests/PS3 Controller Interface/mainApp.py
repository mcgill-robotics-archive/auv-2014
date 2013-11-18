"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
from UIFeedback import *
from PyQt4 import QtCore, QtGui
import sys

updateFrequency = 50


class Main(QtGui.QMainWindow):
    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.timer = QtCore.QTimer()
    #create list of buttons as checkboxes
        self.ps3 = PS3Controller.PS3Controller()
        self.checkboxList = (self.ui.select, self.ui.l3, self.ui.r3, self.ui.start, self.ui.up, self.ui.right, self.ui.down, self.ui.left, self.ui.l2, self.ui.r2, self.ui.l1, self.ui.r1, self.ui.triangle, self.ui.o, self.ui.x, self.ui.square)
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)

        #buttons
        QtCore.QObject.connect(self.ui.changeStatus, QtCore.SIGNAL("clicked()"), self.setTimer)

        # constant timer
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self.controllerUpdate)

    def setTimer(self):
        """
        Start the timer
        """
        if self.ui.ManualRB.isChecked():
            self.timer.start(updateFrequency)
        elif self.ui.AutonomusRB.isChecked():
            self.timer.stop()

    def controllerUpdate(self):
        """
        slot for timer timeout
        """
        #update the state of the controller
        self.ps3.updateController()

        #react to the joysticks
        self.ui.verticalSlider.setValue(-100*self.ps3.lJoyY)
        self.ui.horizontalSlider.setValue(100*self.ps3.lJoyX)
        self.ui.verticalSlider_2.setValue(-100*self.ps3.rJoyY)
        self.ui.horizontalSlider_3.setValue(100*self.ps3.rJoyX)

        #react to the buttons
        buttonState = self.ps3.returnButtons()
        for i in range(0, len(buttonState), 1):
            if buttonState[i] == 1:
                self.checkboxList[i].setChecked(True)
            else:
                self.checkboxList[i].setChecked(False)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = Main()
    AppWindow.show()
    sys.exit(app.exec_())