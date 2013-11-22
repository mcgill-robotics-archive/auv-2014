#!/usr/bin/env python
"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
from UIFeedback import *
from PyQt4 import QtCore, QtGui
import ps3_publisher
import sys
import numpy

updateFrequency = 50


class Main(QtGui.QMainWindow):
    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.controller_timer = QtCore.QTimer()
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.start(50)

        #create tuple list of checkboxes
        self.ps3 = PS3Controller.PS3Controller()
        self.checkboxList = (self.ui.select, self.ui.l3, self.ui.r3, self.ui.start, self.ui.l2, self.ui.r2, self.ui.l1, self.ui.r1, self.ui.triangle, self.ui.o, self.ui.x, self.ui.square)

        #FILL THE PLOT
        p2 = self.ui.graphicsView.addPlot(title="Multiple curves")
        p2.plot(numpy.random.normal(size=100), pen=(255,0,0))
        p2.plot(numpy.random.normal(size=100)+5, pen=(0,255,0))
        p2.plot(numpy.random.normal(size=100)+10, pen=(0,0,255))

        #new row
        self.ui.graphicsView.nextRow()

#UPDATING PLOT
        global med_curve_data, med_curve, updating_plot
        med_curve_data = [1]
        updating_plot = self.ui.graphicsView.addPlot(title = "Updating plot")
        med_curve = updating_plot.plot(pen = "r")




        #buttons
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.changeStatus, QtCore.SIGNAL("clicked()"), self.setTimer)

        #timer connect
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.controllerUpdate)
        QtCore.QObject.connect(self.plot_timer, QtCore.SIGNAL("timeout()"), self.plot_update)

    def setTimer(self):
        """
        Start the timer
        """
        if self.ui.ManualRB.isChecked():
            self.controller_timer.start(updateFrequency)
        elif self.ui.AutonomusRB.isChecked():
            self.controller_timer.stop()

    def controllerUpdate(self):
        """
        slot for timer timeout
        """
        #update the state of the controller
        self.ps3.updateController()

        #react to the joysticks
        self.ui.verticalSlider.setValue(100*self.ps3.horizontal_front_speed)
        self.ui.horizontalSlider.setValue(-100*self.ps3.horizontal_side_speed)
        self.ui.verticalSlider_2.setValue(-100*self.ps3.pitch_speed)
        self.ui.horizontalSlider_3.setValue(-100*self.ps3.yaw_speed)

        #react to the buttons
        buttonState = self.ps3.returnButtons()
        for i in range(0, len(buttonState), 1):
            if buttonState[i] == 1:
                self.checkboxList[i].setChecked(True)
            else:
                self.checkboxList[i].setChecked(False)
        publisherText = ps3_publisher.ps3_publisher(self.ps3.horizontal_front_speed, self.ps3.horizontal_side_speed, self.ps3.vertical_speed, self.ps3.pitch_speed, self.ps3.yaw_speed, 0)
        self.ui.textField.append(publisherText)

    def plot_update(self):
        global med_curve, med_curve_data, updating_plot
        med_curve_data.append((med_curve_data[len(med_curve_data)-1]+1)*23%31)
        med_curve.setData(med_curve_data)
        updating_plot.setXRange(len(med_curve_data)-100, len(med_curve_data))


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = Main()
    AppWindow.show()
    sys.exit(app.exec_())