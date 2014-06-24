__author__ = 'David'

from PyQt4 import QtCore, QtGui

alarm_file = ""

class Ui_warning(object):
    def setupUi(self, warning):
        warning.setObjectName("warning")
        warning.resize(400, 300)
        warning.setMinimumSize(QtCore.QSize(400, 300))
        warning.setMaximumSize(QtCore.QSize(400, 300))
        warning.setFocusPolicy(QtCore.Qt.StrongFocus)
        warning.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/Images/Only-Logo.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        warning.setWindowIcon(icon)
        self.buttonBox = QtGui.QDialogButtonBox(warning)
        self.buttonBox.setGeometry(QtCore.QRect(280, 240, 91, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setCenterButtons(False)
        self.buttonBox.setObjectName("buttonBox")
        self.label = QtGui.QLabel(warning)
        self.label.setGeometry(QtCore.QRect(30, 40, 301, 91))
        self.label.setObjectName("label")

        self.retranslateUi(warning)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), warning.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), warning.reject)
        QtCore.QMetaObject.connectSlotsByName(warning)

    def retranslateUi(self, warning):
        warning.setWindowTitle(QtGui.QApplication.translate("warning", "List of USB devices", None, QtGui.QApplication.UnicodeUTF8))

## Popup for low battery
#
# little class for displaying a popup when battery reaches critical levels
class BatteryWarningUi(QtGui.QDialog):
    ## The constructor
    #  Loads the ui declaration
    #  @param self The object pointer
    def __init__(self, devices, parent=None):
        super(BatteryWarningUi, self).__init__(parent)
        ## store the ui object
        self.battery_warning_ui = Ui_warning()
        self.battery_warning_ui.setupUi(self)

        QtCore.QObject.connect(self.battery_warning_ui.buttonBox, QtCore.SIGNAL("accepted()"), self.battery_warning_ui.close)

        for i in devices.name:
            self.battery_warning_ui.label.setText(i)

