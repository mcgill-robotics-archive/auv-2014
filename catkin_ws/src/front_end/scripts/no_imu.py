# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'no_imu_gr_resizable.ui'
#
# Created: Tue Feb 25 11:10:44 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1024, 768)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.logObject = QtGui.QTextEdit(self.centralwidget)
        self.logObject.setMinimumSize(QtCore.QSize(0, 0))
        self.logObject.setObjectName(_fromUtf8("logObject"))
        self.verticalLayout_3.addWidget(self.logObject)
        self.vidArray = QtGui.QGridLayout()
        self.vidArray.setSizeConstraint(QtGui.QLayout.SetMaximumSize)
        self.vidArray.setContentsMargins(-1, -1, 0, -1)
        self.vidArray.setHorizontalSpacing(2)
        self.vidArray.setVerticalSpacing(1)
        self.vidArray.setObjectName(_fromUtf8("vidArray"))
        self.postRight = QtGui.QLabel(self.centralwidget)
        self.postRight.setMinimumSize(QtCore.QSize(190, 170))
        self.postRight.setMaximumSize(QtCore.QSize(190, 170))
        self.postRight.setFrameShape(QtGui.QFrame.Box)
        self.postRight.setScaledContents(True)
        self.postRight.setObjectName(_fromUtf8("postRight"))
        self.vidArray.addWidget(self.postRight, 1, 1, 1, 1)
        self.postLeft = QtGui.QLabel(self.centralwidget)
        self.postLeft.setMinimumSize(QtCore.QSize(190, 170))
        self.postLeft.setMaximumSize(QtCore.QSize(190, 170))
        self.postLeft.setFrameShape(QtGui.QFrame.Box)
        self.postLeft.setScaledContents(True)
        self.postLeft.setObjectName(_fromUtf8("postLeft"))
        self.vidArray.addWidget(self.postLeft, 1, 0, 1, 1)
        self.preLeft = QtGui.QLabel(self.centralwidget)
        self.preLeft.setMinimumSize(QtCore.QSize(190, 170))
        self.preLeft.setMaximumSize(QtCore.QSize(190, 170))
        self.preLeft.setFrameShape(QtGui.QFrame.Box)
        self.preLeft.setScaledContents(True)
        self.preLeft.setObjectName(_fromUtf8("preLeft"))
        self.vidArray.addWidget(self.preLeft, 0, 0, 1, 1)
        self.preRight = QtGui.QLabel(self.centralwidget)
        self.preRight.setMinimumSize(QtCore.QSize(190, 170))
        self.preRight.setMaximumSize(QtCore.QSize(190, 170))
        self.preRight.setFrameShape(QtGui.QFrame.Box)
        self.preRight.setScaledContents(True)
        self.preRight.setObjectName(_fromUtf8("preRight"))
        self.vidArray.addWidget(self.preRight, 0, 1, 1, 1)
        self.preBottom = QtGui.QLabel(self.centralwidget)
        self.preBottom.setMinimumSize(QtCore.QSize(190, 170))
        self.preBottom.setMaximumSize(QtCore.QSize(190, 170))
        self.preBottom.setFrameShape(QtGui.QFrame.Box)
        self.preBottom.setScaledContents(True)
        self.preBottom.setObjectName(_fromUtf8("preBottom"))
        self.vidArray.addWidget(self.preBottom, 0, 2, 1, 1)
        self.posBottom = QtGui.QLabel(self.centralwidget)
        self.posBottom.setMinimumSize(QtCore.QSize(190, 170))
        self.posBottom.setMaximumSize(QtCore.QSize(190, 170))
        self.posBottom.setFrameShape(QtGui.QFrame.Box)
        self.posBottom.setScaledContents(True)
        self.posBottom.setObjectName(_fromUtf8("posBottom"))
        self.vidArray.addWidget(self.posBottom, 1, 2, 1, 1)
        self.vidArray.setColumnStretch(0, 1)
        self.vidArray.setRowStretch(0, 1)
        self.verticalLayout_3.addLayout(self.vidArray)
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)
        self.line_4 = QtGui.QFrame(self.centralwidget)
        self.line_4.setFrameShape(QtGui.QFrame.VLine)
        self.line_4.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_4.setObjectName(_fromUtf8("line_4"))
        self.horizontalLayout_2.addWidget(self.line_4)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.verticalLayout_4.addWidget(self.label_4)
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setMinimumSize(QtCore.QSize(419, 292))
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.verticalLayoutWidget = QtGui.QWidget(self.frame)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 421, 291))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.poseVizFrame = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.poseVizFrame.setSizeConstraint(QtGui.QLayout.SetMaximumSize)
        self.poseVizFrame.setMargin(0)
        self.poseVizFrame.setObjectName(_fromUtf8("poseVizFrame"))
        self.verticalLayout_4.addWidget(self.frame)
        self.verticalLayout.addLayout(self.verticalLayout_4)
        self.imugraphics = GraphicsLayoutWidget(self.centralwidget)
        self.imugraphics.setEnabled(True)
        self.imugraphics.setMinimumSize(QtCore.QSize(0, 0))
        self.imugraphics.setMaximumSize(QtCore.QSize(1920, 980))
        self.imugraphics.setObjectName(_fromUtf8("imugraphics"))
        self.verticalLayout.addWidget(self.imugraphics)
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        self.verticalLayout_5.addLayout(self.horizontalLayout_2)
        self.line_3 = QtGui.QFrame(self.centralwidget)
        self.line_3.setFrameShape(QtGui.QFrame.HLine)
        self.line_3.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_3.setObjectName(_fromUtf8("line_3"))
        self.verticalLayout_5.addWidget(self.line_3)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.mr_logo = QtGui.QLabel(self.centralwidget)
        self.mr_logo.setMinimumSize(QtCore.QSize(110, 100))
        self.mr_logo.setMaximumSize(QtCore.QSize(110, 100))
        self.mr_logo.setText(_fromUtf8(""))
        self.mr_logo.setPixmap(QtGui.QPixmap(_fromUtf8(":/Images/Only-Logo.png")))
        self.mr_logo.setScaledContents(True)
        self.mr_logo.setObjectName(_fromUtf8("mr_logo"))
        self.verticalLayout_2.addWidget(self.mr_logo)
        self.bat_lbl = QtGui.QLabel(self.centralwidget)
        self.bat_lbl.setMaximumSize(QtCore.QSize(110, 16777215))
        self.bat_lbl.setObjectName(_fromUtf8("bat_lbl"))
        self.verticalLayout_2.addWidget(self.bat_lbl)
        self.battery_progress = QtGui.QProgressBar(self.centralwidget)
        self.battery_progress.setMaximumSize(QtCore.QSize(110, 16777215))
        self.battery_progress.setProperty("value", 24)
        self.battery_progress.setObjectName(_fromUtf8("battery_progress"))
        self.verticalLayout_2.addWidget(self.battery_progress)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.line = QtGui.QFrame(self.centralwidget)
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.horizontalLayout.addWidget(self.line)
        self.linear_cmd_vel = QtGui.QGridLayout()
        self.linear_cmd_vel.setObjectName(_fromUtf8("linear_cmd_vel"))
        self.linearHorizantal = QtGui.QSlider(self.centralwidget)
        self.linearHorizantal.setOrientation(QtCore.Qt.Horizontal)
        self.linearHorizantal.setObjectName(_fromUtf8("linearHorizantal"))
        self.linear_cmd_vel.addWidget(self.linearHorizantal, 2, 0, 1, 1)
        self.linearVertical = QtGui.QSlider(self.centralwidget)
        self.linearVertical.setOrientation(QtCore.Qt.Vertical)
        self.linearVertical.setObjectName(_fromUtf8("linearVertical"))
        self.linear_cmd_vel.addWidget(self.linearVertical, 1, 1, 2, 1)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.linearX = QtGui.QLineEdit(self.centralwidget)
        self.linearX.setObjectName(_fromUtf8("linearX"))
        self.gridLayout_2.addWidget(self.linearX, 0, 1, 1, 1)
        self.label_11 = QtGui.QLabel(self.centralwidget)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_2.addWidget(self.label_11, 0, 0, 1, 1)
        self.linearY = QtGui.QLineEdit(self.centralwidget)
        self.linearY.setObjectName(_fromUtf8("linearY"))
        self.gridLayout_2.addWidget(self.linearY, 1, 1, 1, 1)
        self.linearZ = QtGui.QLineEdit(self.centralwidget)
        self.linearZ.setObjectName(_fromUtf8("linearZ"))
        self.gridLayout_2.addWidget(self.linearZ, 2, 1, 1, 1)
        self.label_12 = QtGui.QLabel(self.centralwidget)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_2.addWidget(self.label_12, 1, 0, 1, 1)
        self.label_13 = QtGui.QLabel(self.centralwidget)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_2.addWidget(self.label_13, 2, 0, 1, 1)
        self.linear_cmd_vel.addLayout(self.gridLayout_2, 1, 0, 1, 1)
        self.label_17 = QtGui.QLabel(self.centralwidget)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.linear_cmd_vel.addWidget(self.label_17, 0, 0, 1, 1)
        self.linear_cmd_vel.setColumnStretch(0, 1)
        self.linear_cmd_vel.setColumnStretch(1, 1)
        self.linear_cmd_vel.setRowStretch(0, 1)
        self.linear_cmd_vel.setRowStretch(1, 1)
        self.linear_cmd_vel.setRowStretch(2, 1)
        self.horizontalLayout.addLayout(self.linear_cmd_vel)
        self.angularcmd_vel = QtGui.QGridLayout()
        self.angularcmd_vel.setObjectName(_fromUtf8("angularcmd_vel"))
        self.gridLayout_5 = QtGui.QGridLayout()
        self.gridLayout_5.setObjectName(_fromUtf8("gridLayout_5"))
        self.angularX = QtGui.QLineEdit(self.centralwidget)
        self.angularX.setObjectName(_fromUtf8("angularX"))
        self.gridLayout_5.addWidget(self.angularX, 0, 1, 1, 1)
        self.label_14 = QtGui.QLabel(self.centralwidget)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.gridLayout_5.addWidget(self.label_14, 0, 0, 1, 1)
        self.angularY = QtGui.QLineEdit(self.centralwidget)
        self.angularY.setObjectName(_fromUtf8("angularY"))
        self.gridLayout_5.addWidget(self.angularY, 1, 1, 1, 1)
        self.angularZ = QtGui.QLineEdit(self.centralwidget)
        self.angularZ.setObjectName(_fromUtf8("angularZ"))
        self.gridLayout_5.addWidget(self.angularZ, 2, 1, 1, 1)
        self.label_15 = QtGui.QLabel(self.centralwidget)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.gridLayout_5.addWidget(self.label_15, 1, 0, 1, 1)
        self.label_16 = QtGui.QLabel(self.centralwidget)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.gridLayout_5.addWidget(self.label_16, 2, 0, 1, 1)
        self.angularcmd_vel.addLayout(self.gridLayout_5, 1, 0, 1, 1)
        self.angularHorizantal = QtGui.QSlider(self.centralwidget)
        self.angularHorizantal.setOrientation(QtCore.Qt.Horizontal)
        self.angularHorizantal.setObjectName(_fromUtf8("angularHorizantal"))
        self.angularcmd_vel.addWidget(self.angularHorizantal, 2, 0, 1, 1)
        self.label_18 = QtGui.QLabel(self.centralwidget)
        self.label_18.setObjectName(_fromUtf8("label_18"))
        self.angularcmd_vel.addWidget(self.label_18, 0, 0, 1, 1)
        self.angularVertical = QtGui.QSlider(self.centralwidget)
        self.angularVertical.setOrientation(QtCore.Qt.Vertical)
        self.angularVertical.setObjectName(_fromUtf8("angularVertical"))
        self.angularcmd_vel.addWidget(self.angularVertical, 1, 1, 2, 1)
        self.horizontalLayout.addLayout(self.angularcmd_vel)
        self.line_5 = QtGui.QFrame(self.centralwidget)
        self.line_5.setFrameShape(QtGui.QFrame.VLine)
        self.line_5.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_5.setObjectName(_fromUtf8("line_5"))
        self.horizontalLayout.addWidget(self.line_5)
        self.verticalLayout_7 = QtGui.QVBoxLayout()
        self.verticalLayout_7.setObjectName(_fromUtf8("verticalLayout_7"))
        self.label_7 = QtGui.QLabel(self.centralwidget)
        self.label_7.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.verticalLayout_7.addWidget(self.label_7)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label_10 = QtGui.QLabel(self.centralwidget)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout.addWidget(self.label_10, 2, 0, 1, 1)
        self.cv_rel_y = QtGui.QLineEdit(self.centralwidget)
        self.cv_rel_y.setObjectName(_fromUtf8("cv_rel_y"))
        self.gridLayout.addWidget(self.cv_rel_y, 1, 1, 1, 1)
        self.cv_rel_z = QtGui.QLineEdit(self.centralwidget)
        self.cv_rel_z.setObjectName(_fromUtf8("cv_rel_z"))
        self.gridLayout.addWidget(self.cv_rel_z, 2, 1, 1, 1)
        self.cv_rel_x = QtGui.QLineEdit(self.centralwidget)
        self.cv_rel_x.setObjectName(_fromUtf8("cv_rel_x"))
        self.gridLayout.addWidget(self.cv_rel_x, 0, 1, 1, 1)
        self.label_8 = QtGui.QLabel(self.centralwidget)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout.addWidget(self.label_8, 0, 0, 1, 1)
        self.label_9 = QtGui.QLabel(self.centralwidget)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout.addWidget(self.label_9, 1, 0, 1, 1)
        self.verticalLayout_7.addLayout(self.gridLayout)
        self.horizontalLayout.addLayout(self.verticalLayout_7)
        self.cv_data = QtGui.QVBoxLayout()
        self.cv_data.setObjectName(_fromUtf8("cv_data"))
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.cv_data.addWidget(self.label_6)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.label_20 = QtGui.QLabel(self.centralwidget)
        self.label_20.setObjectName(_fromUtf8("label_20"))
        self.gridLayout_3.addWidget(self.label_20, 1, 0, 1, 1)
        self.label_21 = QtGui.QLabel(self.centralwidget)
        self.label_21.setObjectName(_fromUtf8("label_21"))
        self.gridLayout_3.addWidget(self.label_21, 0, 0, 1, 1)
        self.cv_rel_pitch = QtGui.QLineEdit(self.centralwidget)
        self.cv_rel_pitch.setObjectName(_fromUtf8("cv_rel_pitch"))
        self.gridLayout_3.addWidget(self.cv_rel_pitch, 0, 1, 1, 1)
        self.cv_rel_yaw = QtGui.QLineEdit(self.centralwidget)
        self.cv_rel_yaw.setObjectName(_fromUtf8("cv_rel_yaw"))
        self.gridLayout_3.addWidget(self.cv_rel_yaw, 1, 1, 1, 1)
        self.cv_data.addLayout(self.gridLayout_3)
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.cv_data.addWidget(self.label_5)
        self.resSelect = QtGui.QComboBox(self.centralwidget)
        self.resSelect.setObjectName(_fromUtf8("resSelect"))
        self.resSelect.addItem(_fromUtf8(""))
        self.resSelect.addItem(_fromUtf8(""))
        self.resSelect.addItem(_fromUtf8(""))
        self.cv_data.addWidget(self.resSelect)
        self.horizontalLayout.addLayout(self.cv_data)
        self.line_2 = QtGui.QFrame(self.centralwidget)
        self.line_2.setFrameShape(QtGui.QFrame.VLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.horizontalLayout.addWidget(self.line_2)
        self.controller_select = QtGui.QVBoxLayout()
        self.controller_select.setObjectName(_fromUtf8("controller_select"))
        self.label_19 = QtGui.QLabel(self.centralwidget)
        self.label_19.setMinimumSize(QtCore.QSize(115, 0))
        self.label_19.setMaximumSize(QtCore.QSize(115, 16777215))
        self.label_19.setObjectName(_fromUtf8("label_19"))
        self.controller_select.addWidget(self.label_19)
        self.keyboardControl = QtGui.QRadioButton(self.centralwidget)
        self.keyboardControl.setMinimumSize(QtCore.QSize(115, 0))
        self.keyboardControl.setMaximumSize(QtCore.QSize(115, 16777215))
        self.keyboardControl.setObjectName(_fromUtf8("keyboardControl"))
        self.controller_select.addWidget(self.keyboardControl)
        self.manualControl = QtGui.QRadioButton(self.centralwidget)
        self.manualControl.setMinimumSize(QtCore.QSize(115, 0))
        self.manualControl.setMaximumSize(QtCore.QSize(115, 16777215))
        self.manualControl.setObjectName(_fromUtf8("manualControl"))
        self.controller_select.addWidget(self.manualControl)
        self.autonomousControl = QtGui.QRadioButton(self.centralwidget)
        self.autonomousControl.setMinimumSize(QtCore.QSize(115, 0))
        self.autonomousControl.setMaximumSize(QtCore.QSize(115, 16777215))
        self.autonomousControl.setObjectName(_fromUtf8("autonomousControl"))
        self.controller_select.addWidget(self.autonomousControl)
        self.attemptPS3 = QtGui.QPushButton(self.centralwidget)
        self.attemptPS3.setMinimumSize(QtCore.QSize(115, 0))
        self.attemptPS3.setMaximumSize(QtCore.QSize(115, 16777215))
        self.attemptPS3.setObjectName(_fromUtf8("attemptPS3"))
        self.controller_select.addWidget(self.attemptPS3)
        self.horizontalLayout.addLayout(self.controller_select)
        self.verticalLayout_5.addLayout(self.horizontalLayout)
        self.verticalLayout_5.setStretch(2, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1024, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        self.postRight.setText(QtGui.QApplication.translate("MainWindow", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.postLeft.setText(QtGui.QApplication.translate("MainWindow", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.preLeft.setText(QtGui.QApplication.translate("MainWindow", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.preRight.setText(QtGui.QApplication.translate("MainWindow", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.preBottom.setText(QtGui.QApplication.translate("MainWindow", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.posBottom.setText(QtGui.QApplication.translate("MainWindow", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("MainWindow", "IMU Pose", None, QtGui.QApplication.UnicodeUTF8))
        self.bat_lbl.setText(QtGui.QApplication.translate("MainWindow", "Battery State", None, QtGui.QApplication.UnicodeUTF8))
        self.label_11.setText(QtGui.QApplication.translate("MainWindow", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.label_12.setText(QtGui.QApplication.translate("MainWindow", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.label_13.setText(QtGui.QApplication.translate("MainWindow", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.label_17.setText(QtGui.QApplication.translate("MainWindow", "Linear", None, QtGui.QApplication.UnicodeUTF8))
        self.label_14.setText(QtGui.QApplication.translate("MainWindow", "Roll", None, QtGui.QApplication.UnicodeUTF8))
        self.label_15.setText(QtGui.QApplication.translate("MainWindow", "Pitch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_16.setText(QtGui.QApplication.translate("MainWindow", "Yaw", None, QtGui.QApplication.UnicodeUTF8))
        self.label_18.setText(QtGui.QApplication.translate("MainWindow", "Angular", None, QtGui.QApplication.UnicodeUTF8))
        self.label_7.setText(QtGui.QApplication.translate("MainWindow", "Relative distance to object", None, QtGui.QApplication.UnicodeUTF8))
        self.label_10.setText(QtGui.QApplication.translate("MainWindow", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.label_8.setText(QtGui.QApplication.translate("MainWindow", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.label_9.setText(QtGui.QApplication.translate("MainWindow", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("MainWindow", "Relative angle to object", None, QtGui.QApplication.UnicodeUTF8))
        self.label_20.setText(QtGui.QApplication.translate("MainWindow", "Yaw", None, QtGui.QApplication.UnicodeUTF8))
        self.label_21.setText(QtGui.QApplication.translate("MainWindow", "Pitch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("MainWindow", "Approximate resolution", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setToolTip(QtGui.QApplication.translate("MainWindow", "<html><head/><body><p>Approximate resolution to set the size of the video displays</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setItemText(0, QtGui.QApplication.translate("MainWindow", "1024*768", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setItemText(1, QtGui.QApplication.translate("MainWindow", "1280*720", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setItemText(2, QtGui.QApplication.translate("MainWindow", "1920x1080", None, QtGui.QApplication.UnicodeUTF8))
        self.label_19.setText(QtGui.QApplication.translate("MainWindow", "Select Controller", None, QtGui.QApplication.UnicodeUTF8))
        self.keyboardControl.setText(QtGui.QApplication.translate("MainWindow", "Keyboard", None, QtGui.QApplication.UnicodeUTF8))
        self.manualControl.setText(QtGui.QApplication.translate("MainWindow", "PS3", None, QtGui.QApplication.UnicodeUTF8))
        self.autonomousControl.setText(QtGui.QApplication.translate("MainWindow", "Autonomus", None, QtGui.QApplication.UnicodeUTF8))
        self.attemptPS3.setText(QtGui.QApplication.translate("MainWindow", "Change", None, QtGui.QApplication.UnicodeUTF8))

from pyqtgraph import GraphicsLayoutWidget
import Robotics_graphical_ressources_rc
