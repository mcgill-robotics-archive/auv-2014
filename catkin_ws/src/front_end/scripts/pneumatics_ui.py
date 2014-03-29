# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Pneumatics.ui'
#
# Created: Sat Mar 29 13:10:20 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Pnematic(object):
    def setupUi(self, Pnematic):
        Pnematic.setObjectName(_fromUtf8("Pnematic"))
        Pnematic.resize(290, 272)
        Pnematic.setMinimumSize(QtCore.QSize(189, 259))
        Pnematic.setMaximumSize(QtCore.QSize(15555, 155555))
        self.centralwidget = QtGui.QWidget(Pnematic)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout_2 = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.tor1 = QtGui.QPushButton(self.centralwidget)
        self.tor1.setObjectName(_fromUtf8("tor1"))
        self.gridLayout.addWidget(self.tor1, 0, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.tor2 = QtGui.QPushButton(self.centralwidget)
        self.tor2.setObjectName(_fromUtf8("tor2"))
        self.gridLayout.addWidget(self.tor2, 1, 1, 1, 1)
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)
        self.mar1 = QtGui.QPushButton(self.centralwidget)
        self.mar1.setObjectName(_fromUtf8("mar1"))
        self.gridLayout.addWidget(self.mar1, 2, 1, 1, 1)
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout.addWidget(self.label_4, 3, 0, 1, 1)
        self.mar2 = QtGui.QPushButton(self.centralwidget)
        self.mar2.setObjectName(_fromUtf8("mar2"))
        self.gridLayout.addWidget(self.mar2, 3, 1, 1, 1)
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout.addWidget(self.label_5, 4, 0, 1, 1)
        self.grab1_open = QtGui.QPushButton(self.centralwidget)
        self.grab1_open.setObjectName(_fromUtf8("grab1_open"))
        self.gridLayout.addWidget(self.grab1_open, 4, 1, 1, 1)
        self.grab1_close = QtGui.QPushButton(self.centralwidget)
        self.grab1_close.setObjectName(_fromUtf8("grab1_close"))
        self.gridLayout.addWidget(self.grab1_close, 4, 2, 1, 1)
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout.addWidget(self.label_6, 5, 0, 1, 1)
        self.grab2_open = QtGui.QPushButton(self.centralwidget)
        self.grab2_open.setObjectName(_fromUtf8("grab2_open"))
        self.gridLayout.addWidget(self.grab2_open, 5, 1, 1, 1)
        self.grab2_close = QtGui.QPushButton(self.centralwidget)
        self.grab2_close.setObjectName(_fromUtf8("grab2_close"))
        self.gridLayout.addWidget(self.grab2_close, 5, 2, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)
        Pnematic.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(Pnematic)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 290, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        Pnematic.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(Pnematic)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        Pnematic.setStatusBar(self.statusbar)
        self.toolBar = QtGui.QToolBar(Pnematic)
        self.toolBar.setObjectName(_fromUtf8("toolBar"))
        Pnematic.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)

        self.retranslateUi(Pnematic)
        QtCore.QMetaObject.connectSlotsByName(Pnematic)

    def retranslateUi(self, Pnematic):
        Pnematic.setWindowTitle(QtGui.QApplication.translate("Pnematic", "Pnematic", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Pnematic", "Torpedo 1", None, QtGui.QApplication.UnicodeUTF8))
        self.tor1.setText(QtGui.QApplication.translate("Pnematic", "Launch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("Pnematic", "Torpedo 2", None, QtGui.QApplication.UnicodeUTF8))
        self.tor2.setText(QtGui.QApplication.translate("Pnematic", "Launch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("Pnematic", "Marker 1", None, QtGui.QApplication.UnicodeUTF8))
        self.mar1.setText(QtGui.QApplication.translate("Pnematic", "Release", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("Pnematic", "Marker 2", None, QtGui.QApplication.UnicodeUTF8))
        self.mar2.setText(QtGui.QApplication.translate("Pnematic", "Release", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("Pnematic", "Grabber 1", None, QtGui.QApplication.UnicodeUTF8))
        self.grab1_open.setText(QtGui.QApplication.translate("Pnematic", "Open", None, QtGui.QApplication.UnicodeUTF8))
        self.grab1_close.setText(QtGui.QApplication.translate("Pnematic", "Close", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("Pnematic", "Grabber 2", None, QtGui.QApplication.UnicodeUTF8))
        self.grab2_open.setText(QtGui.QApplication.translate("Pnematic", "Open", None, QtGui.QApplication.UnicodeUTF8))
        self.grab2_close.setText(QtGui.QApplication.translate("Pnematic", "Close", None, QtGui.QApplication.UnicodeUTF8))
        self.toolBar.setWindowTitle(QtGui.QApplication.translate("Pnematic", "toolBar", None, QtGui.QApplication.UnicodeUTF8))

