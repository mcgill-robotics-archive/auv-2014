# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'thruster_control.ui'
#
# Created: Wed Apr  2 21:07:45 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Thrust_Ui(object):
    def setupUi(self, Thrust_Ui):
        Thrust_Ui.setObjectName(_fromUtf8("Thrust_Ui"))
        Thrust_Ui.resize(620, 358)
        Thrust_Ui.setMaximumSize(QtCore.QSize(16777215, 16777215))
        Thrust_Ui.setAnimated(True)
        self.centralwidget = QtGui.QWidget(Thrust_Ui)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout_5 = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout_5.setObjectName(_fromUtf8("gridLayout_5"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.label_7 = QtGui.QLabel(self.centralwidget)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.verticalLayout_5.addWidget(self.label_7)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.fiel_thruster_3 = QtGui.QSpinBox(self.centralwidget)
        self.fiel_thruster_3.setEnabled(True)
        self.fiel_thruster_3.setReadOnly(True)
        self.fiel_thruster_3.setMinimum(-500)
        self.fiel_thruster_3.setMaximum(500)
        self.fiel_thruster_3.setObjectName(_fromUtf8("fiel_thruster_3"))
        self.gridLayout.addWidget(self.fiel_thruster_3, 3, 1, 1, 1)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setEnabled(False)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.fiel_thruster_2 = QtGui.QSpinBox(self.centralwidget)
        self.fiel_thruster_2.setEnabled(True)
        self.fiel_thruster_2.setReadOnly(True)
        self.fiel_thruster_2.setMinimum(-500)
        self.fiel_thruster_2.setMaximum(500)
        self.fiel_thruster_2.setObjectName(_fromUtf8("fiel_thruster_2"))
        self.gridLayout.addWidget(self.fiel_thruster_2, 2, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setEnabled(False)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout.addWidget(self.label_2, 2, 0, 1, 1)
        self.fiel_thruster_5 = QtGui.QSpinBox(self.centralwidget)
        self.fiel_thruster_5.setEnabled(True)
        self.fiel_thruster_5.setReadOnly(True)
        self.fiel_thruster_5.setMinimum(-500)
        self.fiel_thruster_5.setMaximum(500)
        self.fiel_thruster_5.setObjectName(_fromUtf8("fiel_thruster_5"))
        self.gridLayout.addWidget(self.fiel_thruster_5, 5, 1, 1, 1)
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setEnabled(False)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout.addWidget(self.label_3, 3, 0, 1, 1)
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setEnabled(False)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout.addWidget(self.label_6, 6, 0, 1, 1)
        self.fiel_thruster_4 = QtGui.QSpinBox(self.centralwidget)
        self.fiel_thruster_4.setEnabled(True)
        self.fiel_thruster_4.setReadOnly(True)
        self.fiel_thruster_4.setMinimum(-500)
        self.fiel_thruster_4.setMaximum(500)
        self.fiel_thruster_4.setObjectName(_fromUtf8("fiel_thruster_4"))
        self.gridLayout.addWidget(self.fiel_thruster_4, 4, 1, 1, 1)
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setEnabled(False)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout.addWidget(self.label_5, 5, 0, 1, 1)
        self.fiel_thruster_6 = QtGui.QSpinBox(self.centralwidget)
        self.fiel_thruster_6.setEnabled(True)
        self.fiel_thruster_6.setReadOnly(True)
        self.fiel_thruster_6.setMinimum(-500)
        self.fiel_thruster_6.setMaximum(500)
        self.fiel_thruster_6.setObjectName(_fromUtf8("fiel_thruster_6"))
        self.gridLayout.addWidget(self.fiel_thruster_6, 6, 1, 1, 1)
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setEnabled(False)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout.addWidget(self.label_4, 4, 0, 1, 1)
        self.fiel_thruste_1 = QtGui.QSpinBox(self.centralwidget)
        self.fiel_thruste_1.setEnabled(True)
        self.fiel_thruste_1.setReadOnly(True)
        self.fiel_thruste_1.setMinimum(-500)
        self.fiel_thruste_1.setMaximum(500)
        self.fiel_thruste_1.setObjectName(_fromUtf8("fiel_thruste_1"))
        self.gridLayout.addWidget(self.fiel_thruste_1, 1, 1, 1, 1)
        self.verticalLayout_5.addLayout(self.gridLayout)
        self.horizontalLayout.addLayout(self.verticalLayout_5)
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.thruster_stop = QtGui.QCheckBox(self.centralwidget)
        self.thruster_stop.setCheckable(True)
        self.thruster_stop.setObjectName(_fromUtf8("thruster_stop"))
        self.verticalLayout_6.addWidget(self.thruster_stop)
        self.ps3_present = QtGui.QCheckBox(self.centralwidget)
        self.ps3_present.setCheckable(True)
        self.ps3_present.setTristate(False)
        self.ps3_present.setObjectName(_fromUtf8("ps3_present"))
        self.verticalLayout_6.addWidget(self.ps3_present)
        self.horizontalLayout.addLayout(self.verticalLayout_6)
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.label_14 = QtGui.QLabel(self.centralwidget)
        self.label_14.setEnabled(True)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.verticalLayout_3.addWidget(self.label_14)
        self.gridLayout_4 = QtGui.QGridLayout()
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.label_15 = QtGui.QLabel(self.centralwidget)
        self.label_15.setEnabled(True)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.gridLayout_4.addWidget(self.label_15, 0, 0, 1, 1)
        self.x_force = QtGui.QSpinBox(self.centralwidget)
        self.x_force.setEnabled(True)
        self.x_force.setMinimum(-500)
        self.x_force.setMaximum(500)
        self.x_force.setObjectName(_fromUtf8("x_force"))
        self.gridLayout_4.addWidget(self.x_force, 0, 1, 1, 1)
        self.slider_thruster_15 = QtGui.QSlider(self.centralwidget)
        self.slider_thruster_15.setEnabled(True)
        self.slider_thruster_15.setMinimum(-500)
        self.slider_thruster_15.setMaximum(500)
        self.slider_thruster_15.setOrientation(QtCore.Qt.Horizontal)
        self.slider_thruster_15.setObjectName(_fromUtf8("slider_thruster_15"))
        self.gridLayout_4.addWidget(self.slider_thruster_15, 0, 2, 1, 1)
        self.label_16 = QtGui.QLabel(self.centralwidget)
        self.label_16.setEnabled(True)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.gridLayout_4.addWidget(self.label_16, 1, 0, 1, 1)
        self.x_bal = QtGui.QSpinBox(self.centralwidget)
        self.x_bal.setEnabled(True)
        self.x_bal.setMinimum(-100)
        self.x_bal.setMaximum(100)
        self.x_bal.setObjectName(_fromUtf8("x_bal"))
        self.gridLayout_4.addWidget(self.x_bal, 1, 1, 1, 1)
        self.slider_thruster_16 = QtGui.QSlider(self.centralwidget)
        self.slider_thruster_16.setEnabled(True)
        self.slider_thruster_16.setMinimum(-100)
        self.slider_thruster_16.setMaximum(100)
        self.slider_thruster_16.setOrientation(QtCore.Qt.Horizontal)
        self.slider_thruster_16.setObjectName(_fromUtf8("slider_thruster_16"))
        self.gridLayout_4.addWidget(self.slider_thruster_16, 1, 2, 1, 1)
        self.verticalLayout_3.addLayout(self.gridLayout_4)
        self.verticalLayout_2.addLayout(self.verticalLayout_3)
        self.label_11 = QtGui.QLabel(self.centralwidget)
        self.label_11.setEnabled(True)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.verticalLayout_2.addWidget(self.label_11)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.label_12 = QtGui.QLabel(self.centralwidget)
        self.label_12.setEnabled(True)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_3.addWidget(self.label_12, 0, 0, 1, 1)
        self.y_force = QtGui.QSpinBox(self.centralwidget)
        self.y_force.setEnabled(True)
        self.y_force.setMinimum(-500)
        self.y_force.setMaximum(500)
        self.y_force.setObjectName(_fromUtf8("y_force"))
        self.gridLayout_3.addWidget(self.y_force, 0, 1, 1, 1)
        self.slider_thruster_13 = QtGui.QSlider(self.centralwidget)
        self.slider_thruster_13.setEnabled(True)
        self.slider_thruster_13.setMinimum(-500)
        self.slider_thruster_13.setMaximum(500)
        self.slider_thruster_13.setOrientation(QtCore.Qt.Horizontal)
        self.slider_thruster_13.setObjectName(_fromUtf8("slider_thruster_13"))
        self.gridLayout_3.addWidget(self.slider_thruster_13, 0, 2, 1, 1)
        self.label_13 = QtGui.QLabel(self.centralwidget)
        self.label_13.setEnabled(True)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_3.addWidget(self.label_13, 1, 0, 1, 1)
        self.y_bal = QtGui.QSpinBox(self.centralwidget)
        self.y_bal.setEnabled(True)
        self.y_bal.setMinimum(-100)
        self.y_bal.setMaximum(100)
        self.y_bal.setObjectName(_fromUtf8("y_bal"))
        self.gridLayout_3.addWidget(self.y_bal, 1, 1, 1, 1)
        self.slider_thruster_14 = QtGui.QSlider(self.centralwidget)
        self.slider_thruster_14.setEnabled(True)
        self.slider_thruster_14.setMinimum(-100)
        self.slider_thruster_14.setMaximum(100)
        self.slider_thruster_14.setOrientation(QtCore.Qt.Horizontal)
        self.slider_thruster_14.setObjectName(_fromUtf8("slider_thruster_14"))
        self.gridLayout_3.addWidget(self.slider_thruster_14, 1, 2, 1, 1)
        self.verticalLayout_2.addLayout(self.gridLayout_3)
        self.verticalLayout_4.addLayout(self.verticalLayout_2)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label_10 = QtGui.QLabel(self.centralwidget)
        self.label_10.setEnabled(True)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.verticalLayout.addWidget(self.label_10)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.label_8 = QtGui.QLabel(self.centralwidget)
        self.label_8.setEnabled(True)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout_2.addWidget(self.label_8, 0, 0, 1, 1)
        self.z_force = QtGui.QSpinBox(self.centralwidget)
        self.z_force.setEnabled(True)
        self.z_force.setMinimum(-500)
        self.z_force.setMaximum(500)
        self.z_force.setObjectName(_fromUtf8("z_force"))
        self.gridLayout_2.addWidget(self.z_force, 0, 1, 1, 1)
        self.slider_thruster_12 = QtGui.QSlider(self.centralwidget)
        self.slider_thruster_12.setEnabled(True)
        self.slider_thruster_12.setMinimum(-500)
        self.slider_thruster_12.setMaximum(500)
        self.slider_thruster_12.setOrientation(QtCore.Qt.Horizontal)
        self.slider_thruster_12.setObjectName(_fromUtf8("slider_thruster_12"))
        self.gridLayout_2.addWidget(self.slider_thruster_12, 0, 2, 1, 1)
        self.label_9 = QtGui.QLabel(self.centralwidget)
        self.label_9.setEnabled(True)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_2.addWidget(self.label_9, 1, 0, 1, 1)
        self.z_bal = QtGui.QSpinBox(self.centralwidget)
        self.z_bal.setEnabled(True)
        self.z_bal.setMinimum(-100)
        self.z_bal.setMaximum(100)
        self.z_bal.setObjectName(_fromUtf8("z_bal"))
        self.gridLayout_2.addWidget(self.z_bal, 1, 1, 1, 1)
        self.slider_thruster_11 = QtGui.QSlider(self.centralwidget)
        self.slider_thruster_11.setEnabled(True)
        self.slider_thruster_11.setMinimum(-100)
        self.slider_thruster_11.setMaximum(100)
        self.slider_thruster_11.setOrientation(QtCore.Qt.Horizontal)
        self.slider_thruster_11.setObjectName(_fromUtf8("slider_thruster_11"))
        self.gridLayout_2.addWidget(self.slider_thruster_11, 1, 2, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_2)
        self.verticalLayout_4.addLayout(self.verticalLayout)
        self.horizontalLayout.addLayout(self.verticalLayout_4)
        self.gridLayout_5.addLayout(self.horizontalLayout, 0, 0, 1, 1)
        Thrust_Ui.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(Thrust_Ui)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 620, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName(_fromUtf8("menuFile"))
        Thrust_Ui.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(Thrust_Ui)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        Thrust_Ui.setStatusBar(self.statusbar)
        self.actionQuit = QtGui.QAction(Thrust_Ui)
        self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
        self.menuFile.addAction(self.actionQuit)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(Thrust_Ui)
        QtCore.QObject.connect(self.slider_thruster_15, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x_force.setValue)
        QtCore.QObject.connect(self.x_force, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.slider_thruster_15.setValue)
        QtCore.QObject.connect(self.x_bal, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.slider_thruster_16.setValue)
        QtCore.QObject.connect(self.slider_thruster_16, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x_bal.setValue)
        QtCore.QObject.connect(self.slider_thruster_13, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y_force.setValue)
        QtCore.QObject.connect(self.y_force, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.slider_thruster_13.setValue)
        QtCore.QObject.connect(self.y_bal, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.slider_thruster_14.setValue)
        QtCore.QObject.connect(self.slider_thruster_14, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y_bal.setValue)
        QtCore.QObject.connect(self.z_force, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.slider_thruster_12.setValue)
        QtCore.QObject.connect(self.z_bal, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.slider_thruster_11.setValue)
        QtCore.QObject.connect(self.slider_thruster_12, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.z_force.setValue)
        QtCore.QObject.connect(self.slider_thruster_11, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.z_bal.setValue)
        QtCore.QObject.connect(self.actionQuit, QtCore.SIGNAL(_fromUtf8("triggered()")), Thrust_Ui.close)
        QtCore.QMetaObject.connectSlotsByName(Thrust_Ui)

    def retranslateUi(self, Thrust_Ui):
        Thrust_Ui.setWindowTitle(QtGui.QApplication.translate("Thrust_Ui", "Thruster controller", None, QtGui.QApplication.UnicodeUTF8))
        self.label_7.setText(QtGui.QApplication.translate("Thrust_Ui", "Thruster Speed", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Thrust_Ui", "x1", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("Thrust_Ui", "x2", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("Thrust_Ui", "y1", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("Thrust_Ui", "z2", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("Thrust_Ui", "z1", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("Thrust_Ui", "y2", None, QtGui.QApplication.UnicodeUTF8))
        self.thruster_stop.setText(QtGui.QApplication.translate("Thrust_Ui", "Thruster stop", None, QtGui.QApplication.UnicodeUTF8))
        self.ps3_present.setText(QtGui.QApplication.translate("Thrust_Ui", "PS3 Controller", None, QtGui.QApplication.UnicodeUTF8))
        self.label_14.setText(QtGui.QApplication.translate("Thrust_Ui", "X- Axis", None, QtGui.QApplication.UnicodeUTF8))
        self.label_15.setText(QtGui.QApplication.translate("Thrust_Ui", "Force", None, QtGui.QApplication.UnicodeUTF8))
        self.label_16.setText(QtGui.QApplication.translate("Thrust_Ui", "Balance", None, QtGui.QApplication.UnicodeUTF8))
        self.label_11.setText(QtGui.QApplication.translate("Thrust_Ui", "Y- Axis", None, QtGui.QApplication.UnicodeUTF8))
        self.label_12.setText(QtGui.QApplication.translate("Thrust_Ui", "Force", None, QtGui.QApplication.UnicodeUTF8))
        self.label_13.setText(QtGui.QApplication.translate("Thrust_Ui", "Balance", None, QtGui.QApplication.UnicodeUTF8))
        self.label_10.setText(QtGui.QApplication.translate("Thrust_Ui", "Z- Axis", None, QtGui.QApplication.UnicodeUTF8))
        self.label_8.setText(QtGui.QApplication.translate("Thrust_Ui", "Force", None, QtGui.QApplication.UnicodeUTF8))
        self.label_9.setText(QtGui.QApplication.translate("Thrust_Ui", "Balance", None, QtGui.QApplication.UnicodeUTF8))
        self.menuFile.setTitle(QtGui.QApplication.translate("Thrust_Ui", "File", None, QtGui.QApplication.UnicodeUTF8))
        self.actionQuit.setText(QtGui.QApplication.translate("Thrust_Ui", "Quit", None, QtGui.QApplication.UnicodeUTF8))
        self.actionQuit.setShortcut(QtGui.QApplication.translate("Thrust_Ui", "Ctrl+C", None, QtGui.QApplication.UnicodeUTF8))

