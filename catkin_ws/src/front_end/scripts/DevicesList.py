__author__ = 'David Lavoie-Boutin'

from PyQt4 import QtCore, QtGui

## Popup for low battery
#
# little class for displaying a popup when battery reaches critical levels
class DevicesList(QtGui.QDialog):
    ## The constructor
    #  Loads the ui declaration
    #  @param self The object pointer
    def __init__(self, parent=None, devices=None):
        super(DevicesList, self).__init__(parent)

        self.initui(devices)

    def initui(self, devices):
        text=''
        for i in devices:
            text += i + '\n'

        self.lbl1 = QtGui.QLabel(text, self)
        self.lbl1.move(15, 10)

        self.setGeometry(300, 300, 600, 600)
        self.setWindowTitle('USB Devices')
        self.show()



