# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'internal_leak_warning.ui'
#
# Created: Sun Dec  8 17:13:58 2013
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_warning(object):
    def setupUi(self, warning):
        warning.setObjectName(_fromUtf8("warning"))
        warning.resize(400, 300)
        warning.setMinimumSize(QtCore.QSize(400, 300))
        warning.setMaximumSize(QtCore.QSize(400, 300))
        warning.setFocusPolicy(QtCore.Qt.StrongFocus)
        warning.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/Images/Only-Logo.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        warning.setWindowIcon(icon)
        self.buttonBox = QtGui.QDialogButtonBox(warning)
        self.buttonBox.setGeometry(QtCore.QRect(280, 240, 91, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setCenterButtons(False)
        self.buttonBox.setObjectName(_fromUtf8("buttonBox"))
        self.label = QtGui.QLabel(warning)
        self.label.setGeometry(QtCore.QRect(30, 40, 221, 91))
        self.label.setObjectName(_fromUtf8("label"))

        self.retranslateUi(warning)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("accepted()")), warning.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("rejected()")), warning.reject)
        QtCore.QMetaObject.connectSlotsByName(warning)

    def retranslateUi(self, warning):
        warning.setWindowTitle(QtGui.QApplication.translate("warning", "Leak Warning", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("warning", "<html><head/><body><p>Internal pressure drop detected</p><p>Initiating re-surfacing protocols</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))

import FinalRobotics_rc
