# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'no_imu_gr_resizable.ui'
#
# Created: Mon Mar 31 12:17:27 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_RoboticsMain(object):
    def setupUi(self, RoboticsMain):
        RoboticsMain.setObjectName(_fromUtf8("RoboticsMain"))
        RoboticsMain.resize(1125, 845)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/Images/Only-Logo.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        RoboticsMain.setWindowIcon(icon)
        self.centralwidget = QtGui.QWidget(RoboticsMain)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout_4 = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.frame_2 = QtGui.QFrame(self.centralwidget)
        self.frame_2.setMaximumSize(QtCore.QSize(16777215, 151))
        self.frame_2.setFrameShape(QtGui.QFrame.NoFrame)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setLineWidth(0)
        self.frame_2.setObjectName(_fromUtf8("frame_2"))
        self.gridLayout_3 = QtGui.QGridLayout(self.frame_2)
        self.gridLayout_3.setSpacing(0)
        self.gridLayout_3.setMargin(0)
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(self.frame_2)
        self.label_2.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.label_3 = QtGui.QLabel(self.frame_2)
        self.label_3.setMaximumSize(QtCore.QSize(282, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.horizontalLayout_2.addWidget(self.label_3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label_7 = QtGui.QLabel(self.frame_2)
        self.label_7.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.horizontalLayout.addWidget(self.label_7)
        self.label_6 = QtGui.QLabel(self.frame_2)
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.horizontalLayout.addWidget(self.label_6)
        self.label_22 = QtGui.QLabel(self.frame_2)
        self.label_22.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_22.setObjectName(_fromUtf8("label_22"))
        self.horizontalLayout.addWidget(self.label_22)
        self.label_26 = QtGui.QLabel(self.frame_2)
        self.label_26.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_26.setObjectName(_fromUtf8("label_26"))
        self.horizontalLayout.addWidget(self.label_26)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label_8 = QtGui.QLabel(self.frame_2)
        self.label_8.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout.addWidget(self.label_8, 0, 0, 1, 1)
        self.front_x = QtGui.QLineEdit(self.frame_2)
        self.front_x.setMaximumSize(QtCore.QSize(16777215, 27))
        self.front_x.setObjectName(_fromUtf8("front_x"))
        self.gridLayout.addWidget(self.front_x, 0, 1, 1, 1)
        self.label_21 = QtGui.QLabel(self.frame_2)
        self.label_21.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_21.setObjectName(_fromUtf8("label_21"))
        self.gridLayout.addWidget(self.label_21, 0, 2, 1, 1)
        self.front_pitch = QtGui.QLineEdit(self.frame_2)
        self.front_pitch.setMaximumSize(QtCore.QSize(16777215, 27))
        self.front_pitch.setObjectName(_fromUtf8("front_pitch"))
        self.gridLayout.addWidget(self.front_pitch, 0, 3, 1, 1)
        self.label_24 = QtGui.QLabel(self.frame_2)
        self.label_24.setObjectName(_fromUtf8("label_24"))
        self.gridLayout.addWidget(self.label_24, 0, 4, 1, 1)
        self.down_x = QtGui.QLineEdit(self.frame_2)
        self.down_x.setMaximumSize(QtCore.QSize(16777215, 27))
        self.down_x.setObjectName(_fromUtf8("down_x"))
        self.gridLayout.addWidget(self.down_x, 0, 5, 1, 1)
        self.label_28 = QtGui.QLabel(self.frame_2)
        self.label_28.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_28.setObjectName(_fromUtf8("label_28"))
        self.gridLayout.addWidget(self.label_28, 0, 6, 1, 1)
        self.down_pitch = QtGui.QLineEdit(self.frame_2)
        self.down_pitch.setMaximumSize(QtCore.QSize(16777215, 27))
        self.down_pitch.setObjectName(_fromUtf8("down_pitch"))
        self.gridLayout.addWidget(self.down_pitch, 0, 7, 1, 1)
        self.label_9 = QtGui.QLabel(self.frame_2)
        self.label_9.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout.addWidget(self.label_9, 1, 0, 1, 1)
        self.front_y = QtGui.QLineEdit(self.frame_2)
        self.front_y.setMaximumSize(QtCore.QSize(16777215, 27))
        self.front_y.setObjectName(_fromUtf8("front_y"))
        self.gridLayout.addWidget(self.front_y, 1, 1, 1, 1)
        self.label_20 = QtGui.QLabel(self.frame_2)
        self.label_20.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_20.setObjectName(_fromUtf8("label_20"))
        self.gridLayout.addWidget(self.label_20, 1, 2, 1, 1)
        self.front_yaw = QtGui.QLineEdit(self.frame_2)
        self.front_yaw.setMaximumSize(QtCore.QSize(16777215, 27))
        self.front_yaw.setObjectName(_fromUtf8("front_yaw"))
        self.gridLayout.addWidget(self.front_yaw, 1, 3, 1, 1)
        self.label_25 = QtGui.QLabel(self.frame_2)
        self.label_25.setObjectName(_fromUtf8("label_25"))
        self.gridLayout.addWidget(self.label_25, 1, 4, 1, 1)
        self.down_y = QtGui.QLineEdit(self.frame_2)
        self.down_y.setMaximumSize(QtCore.QSize(16777215, 27))
        self.down_y.setObjectName(_fromUtf8("down_y"))
        self.gridLayout.addWidget(self.down_y, 1, 5, 1, 1)
        self.label_27 = QtGui.QLabel(self.frame_2)
        self.label_27.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_27.setObjectName(_fromUtf8("label_27"))
        self.gridLayout.addWidget(self.label_27, 1, 6, 1, 1)
        self.down_aw = QtGui.QLineEdit(self.frame_2)
        self.down_aw.setMaximumSize(QtCore.QSize(16777215, 27))
        self.down_aw.setObjectName(_fromUtf8("down_aw"))
        self.gridLayout.addWidget(self.down_aw, 1, 7, 1, 1)
        self.label_10 = QtGui.QLabel(self.frame_2)
        self.label_10.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout.addWidget(self.label_10, 2, 0, 1, 1)
        self.front_z = QtGui.QLineEdit(self.frame_2)
        self.front_z.setMaximumSize(QtCore.QSize(16777215, 27))
        self.front_z.setObjectName(_fromUtf8("front_z"))
        self.gridLayout.addWidget(self.front_z, 2, 1, 1, 1)
        self.label = QtGui.QLabel(self.frame_2)
        self.label.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 2, 2, 1, 1)
        self.front_objType = QtGui.QLineEdit(self.frame_2)
        self.front_objType.setMaximumSize(QtCore.QSize(16777215, 27))
        self.front_objType.setObjectName(_fromUtf8("front_objType"))
        self.gridLayout.addWidget(self.front_objType, 2, 3, 1, 1)
        self.label_23 = QtGui.QLabel(self.frame_2)
        self.label_23.setObjectName(_fromUtf8("label_23"))
        self.gridLayout.addWidget(self.label_23, 2, 4, 1, 1)
        self.down_z = QtGui.QLineEdit(self.frame_2)
        self.down_z.setMaximumSize(QtCore.QSize(16777215, 27))
        self.down_z.setObjectName(_fromUtf8("down_z"))
        self.gridLayout.addWidget(self.down_z, 2, 5, 1, 1)
        self.label_29 = QtGui.QLabel(self.frame_2)
        self.label_29.setMaximumSize(QtCore.QSize(16777215, 27))
        self.label_29.setObjectName(_fromUtf8("label_29"))
        self.gridLayout.addWidget(self.label_29, 2, 6, 1, 1)
        self.down_objType = QtGui.QLineEdit(self.frame_2)
        self.down_objType.setMaximumSize(QtCore.QSize(16777215, 27))
        self.down_objType.setObjectName(_fromUtf8("down_objType"))
        self.gridLayout.addWidget(self.down_objType, 2, 7, 1, 1)
        self.verticalLayout_3.addLayout(self.gridLayout)
        self.horizontalLayout_3.addLayout(self.verticalLayout_3)
        self.line_2 = QtGui.QFrame(self.frame_2)
        self.line_2.setFrameShape(QtGui.QFrame.VLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.horizontalLayout_3.addWidget(self.line_2)
        self.linear_cmd_vel = QtGui.QGridLayout()
        self.linear_cmd_vel.setObjectName(_fromUtf8("linear_cmd_vel"))
        self.linearHorizantal = QtGui.QSlider(self.frame_2)
        self.linearHorizantal.setOrientation(QtCore.Qt.Horizontal)
        self.linearHorizantal.setObjectName(_fromUtf8("linearHorizantal"))
        self.linear_cmd_vel.addWidget(self.linearHorizantal, 2, 0, 1, 1)
        self.linearVertical = QtGui.QSlider(self.frame_2)
        self.linearVertical.setOrientation(QtCore.Qt.Vertical)
        self.linearVertical.setObjectName(_fromUtf8("linearVertical"))
        self.linear_cmd_vel.addWidget(self.linearVertical, 1, 1, 2, 1)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.linearX = QtGui.QLineEdit(self.frame_2)
        self.linearX.setObjectName(_fromUtf8("linearX"))
        self.gridLayout_2.addWidget(self.linearX, 0, 1, 1, 1)
        self.label_11 = QtGui.QLabel(self.frame_2)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_2.addWidget(self.label_11, 0, 0, 1, 1)
        self.linearY = QtGui.QLineEdit(self.frame_2)
        self.linearY.setObjectName(_fromUtf8("linearY"))
        self.gridLayout_2.addWidget(self.linearY, 1, 1, 1, 1)
        self.linearZ = QtGui.QLineEdit(self.frame_2)
        self.linearZ.setObjectName(_fromUtf8("linearZ"))
        self.gridLayout_2.addWidget(self.linearZ, 2, 1, 1, 1)
        self.label_12 = QtGui.QLabel(self.frame_2)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_2.addWidget(self.label_12, 1, 0, 1, 1)
        self.label_13 = QtGui.QLabel(self.frame_2)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_2.addWidget(self.label_13, 2, 0, 1, 1)
        self.linear_cmd_vel.addLayout(self.gridLayout_2, 1, 0, 1, 1)
        self.label_17 = QtGui.QLabel(self.frame_2)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.linear_cmd_vel.addWidget(self.label_17, 0, 0, 1, 1)
        self.linear_cmd_vel.setColumnStretch(0, 1)
        self.linear_cmd_vel.setColumnStretch(1, 1)
        self.linear_cmd_vel.setRowStretch(0, 1)
        self.linear_cmd_vel.setRowStretch(1, 3)
        self.linear_cmd_vel.setRowStretch(2, 1)
        self.horizontalLayout_3.addLayout(self.linear_cmd_vel)
        self.angularcmd_vel = QtGui.QGridLayout()
        self.angularcmd_vel.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.angularcmd_vel.setObjectName(_fromUtf8("angularcmd_vel"))
        self.gridLayout_5 = QtGui.QGridLayout()
        self.gridLayout_5.setObjectName(_fromUtf8("gridLayout_5"))
        self.angularX = QtGui.QLineEdit(self.frame_2)
        self.angularX.setObjectName(_fromUtf8("angularX"))
        self.gridLayout_5.addWidget(self.angularX, 0, 1, 1, 1)
        self.label_14 = QtGui.QLabel(self.frame_2)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.gridLayout_5.addWidget(self.label_14, 0, 0, 1, 1)
        self.angularY = QtGui.QLineEdit(self.frame_2)
        self.angularY.setObjectName(_fromUtf8("angularY"))
        self.gridLayout_5.addWidget(self.angularY, 1, 1, 1, 1)
        self.angularZ = QtGui.QLineEdit(self.frame_2)
        self.angularZ.setObjectName(_fromUtf8("angularZ"))
        self.gridLayout_5.addWidget(self.angularZ, 2, 1, 1, 1)
        self.label_15 = QtGui.QLabel(self.frame_2)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.gridLayout_5.addWidget(self.label_15, 1, 0, 1, 1)
        self.label_16 = QtGui.QLabel(self.frame_2)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.gridLayout_5.addWidget(self.label_16, 2, 0, 1, 1)
        self.angularcmd_vel.addLayout(self.gridLayout_5, 1, 0, 1, 1)
        self.angularHorizantal = QtGui.QSlider(self.frame_2)
        self.angularHorizantal.setOrientation(QtCore.Qt.Horizontal)
        self.angularHorizantal.setObjectName(_fromUtf8("angularHorizantal"))
        self.angularcmd_vel.addWidget(self.angularHorizantal, 2, 0, 1, 1)
        self.label_18 = QtGui.QLabel(self.frame_2)
        self.label_18.setObjectName(_fromUtf8("label_18"))
        self.angularcmd_vel.addWidget(self.label_18, 0, 0, 1, 1)
        self.angularVertical = QtGui.QSlider(self.frame_2)
        self.angularVertical.setOrientation(QtCore.Qt.Vertical)
        self.angularVertical.setObjectName(_fromUtf8("angularVertical"))
        self.angularcmd_vel.addWidget(self.angularVertical, 1, 1, 2, 1)
        self.angularcmd_vel.setRowStretch(0, 1)
        self.horizontalLayout_3.addLayout(self.angularcmd_vel)
        self.controller_select = QtGui.QVBoxLayout()
        self.controller_select.setObjectName(_fromUtf8("controller_select"))
        self.label_19 = QtGui.QLabel(self.frame_2)
        self.label_19.setMinimumSize(QtCore.QSize(115, 0))
        self.label_19.setMaximumSize(QtCore.QSize(115, 16777215))
        self.label_19.setObjectName(_fromUtf8("label_19"))
        self.controller_select.addWidget(self.label_19)
        self.keyboardControl = QtGui.QRadioButton(self.frame_2)
        self.keyboardControl.setMinimumSize(QtCore.QSize(115, 0))
        self.keyboardControl.setMaximumSize(QtCore.QSize(115, 16777215))
        self.keyboardControl.setObjectName(_fromUtf8("keyboardControl"))
        self.controller_select.addWidget(self.keyboardControl)
        self.manualControl = QtGui.QRadioButton(self.frame_2)
        self.manualControl.setMinimumSize(QtCore.QSize(115, 0))
        self.manualControl.setMaximumSize(QtCore.QSize(115, 16777215))
        self.manualControl.setObjectName(_fromUtf8("manualControl"))
        self.controller_select.addWidget(self.manualControl)
        self.autonomousControl = QtGui.QRadioButton(self.frame_2)
        self.autonomousControl.setMinimumSize(QtCore.QSize(115, 0))
        self.autonomousControl.setMaximumSize(QtCore.QSize(115, 16777215))
        self.autonomousControl.setObjectName(_fromUtf8("autonomousControl"))
        self.controller_select.addWidget(self.autonomousControl)
        self.attemptPS3 = QtGui.QPushButton(self.frame_2)
        self.attemptPS3.setMinimumSize(QtCore.QSize(115, 0))
        self.attemptPS3.setMaximumSize(QtCore.QSize(115, 16777215))
        self.attemptPS3.setObjectName(_fromUtf8("attemptPS3"))
        self.controller_select.addWidget(self.attemptPS3)
        self.horizontalLayout_3.addLayout(self.controller_select)
        self.gridLayout_3.addLayout(self.horizontalLayout_3, 0, 0, 1, 1)
        self.verticalLayout_6.addWidget(self.frame_2)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.vid_array = QtGui.QGridLayout()
        self.vid_array.setSpacing(2)
        self.vid_array.setObjectName(_fromUtf8("vid_array"))
        self.preLeft = QtGui.QLabel(self.centralwidget)
        self.preLeft.setMinimumSize(QtCore.QSize(250, 230))
        self.preLeft.setMaximumSize(QtCore.QSize(500, 450))
        self.preLeft.setFrameShape(QtGui.QFrame.Box)
        self.preLeft.setScaledContents(True)
        self.preLeft.setObjectName(_fromUtf8("preLeft"))
        self.vid_array.addWidget(self.preLeft, 0, 0, 1, 1)
        self.preRight = QtGui.QLabel(self.centralwidget)
        self.preRight.setMinimumSize(QtCore.QSize(250, 230))
        self.preRight.setMaximumSize(QtCore.QSize(500, 450))
        self.preRight.setFrameShape(QtGui.QFrame.Box)
        self.preRight.setScaledContents(True)
        self.preRight.setObjectName(_fromUtf8("preRight"))
        self.vid_array.addWidget(self.preRight, 1, 0, 1, 1)
        self.postLeft = QtGui.QLabel(self.centralwidget)
        self.postLeft.setMinimumSize(QtCore.QSize(250, 230))
        self.postLeft.setMaximumSize(QtCore.QSize(500, 450))
        self.postLeft.setFrameShape(QtGui.QFrame.Box)
        self.postLeft.setScaledContents(True)
        self.postLeft.setObjectName(_fromUtf8("postLeft"))
        self.vid_array.addWidget(self.postLeft, 0, 1, 1, 1)
        self.preBottom = QtGui.QLabel(self.centralwidget)
        self.preBottom.setMinimumSize(QtCore.QSize(250, 230))
        self.preBottom.setMaximumSize(QtCore.QSize(500, 450))
        self.preBottom.setFrameShape(QtGui.QFrame.Box)
        self.preBottom.setScaledContents(True)
        self.preBottom.setObjectName(_fromUtf8("preBottom"))
        self.vid_array.addWidget(self.preBottom, 2, 0, 1, 1)
        self.postRight = QtGui.QLabel(self.centralwidget)
        self.postRight.setMinimumSize(QtCore.QSize(250, 230))
        self.postRight.setMaximumSize(QtCore.QSize(500, 450))
        self.postRight.setFrameShape(QtGui.QFrame.Box)
        self.postRight.setScaledContents(True)
        self.postRight.setObjectName(_fromUtf8("postRight"))
        self.vid_array.addWidget(self.postRight, 1, 1, 1, 1)
        self.posBottom = QtGui.QLabel(self.centralwidget)
        self.posBottom.setMinimumSize(QtCore.QSize(250, 230))
        self.posBottom.setMaximumSize(QtCore.QSize(500, 450))
        self.posBottom.setFrameShape(QtGui.QFrame.Box)
        self.posBottom.setScaledContents(True)
        self.posBottom.setObjectName(_fromUtf8("posBottom"))
        self.vid_array.addWidget(self.posBottom, 2, 1, 1, 1)
        self.verticalLayout.addLayout(self.vid_array)
        self.horizontalLayout_5.addLayout(self.verticalLayout)
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.label_30 = QtGui.QLabel(self.centralwidget)
        self.label_30.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_30.setObjectName(_fromUtf8("label_30"))
        self.verticalLayout_2.addWidget(self.label_30)
        self.logObject = QtGui.QTextEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.logObject.sizePolicy().hasHeightForWidth())
        self.logObject.setSizePolicy(sizePolicy)
        self.logObject.setMinimumSize(QtCore.QSize(210, 0))
        self.logObject.setMaximumSize(QtCore.QSize(1220223, 12312312))
        self.logObject.setObjectName(_fromUtf8("logObject"))
        self.verticalLayout_2.addWidget(self.logObject)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        self.imugraphics = GraphicsLayoutWidget(self.centralwidget)
        self.imugraphics.setEnabled(True)
        self.imugraphics.setMinimumSize(QtCore.QSize(0, 0))
        self.imugraphics.setMaximumSize(QtCore.QSize(1920, 980))
        self.imugraphics.setObjectName(_fromUtf8("imugraphics"))
        self.horizontalLayout_4.addWidget(self.imugraphics)
        self.verticalLayout_4.addLayout(self.horizontalLayout_4)
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.gridLayout_6 = QtGui.QGridLayout()
        self.gridLayout_6.setObjectName(_fromUtf8("gridLayout_6"))
        self.resSelect = QtGui.QComboBox(self.centralwidget)
        self.resSelect.setObjectName(_fromUtf8("resSelect"))
        self.resSelect.addItem(_fromUtf8(""))
        self.resSelect.addItem(_fromUtf8(""))
        self.resSelect.addItem(_fromUtf8(""))
        self.gridLayout_6.addWidget(self.resSelect, 1, 0, 1, 1)
        self.bat_lcd1 = QtGui.QLCDNumber(self.centralwidget)
        self.bat_lcd1.setObjectName(_fromUtf8("bat_lcd1"))
        self.gridLayout_6.addWidget(self.bat_lcd1, 1, 3, 1, 1)
        self.temp_ind = QtGui.QLCDNumber(self.centralwidget)
        self.temp_ind.setObjectName(_fromUtf8("temp_ind"))
        self.gridLayout_6.addWidget(self.temp_ind, 1, 1, 1, 1)
        self.label_31 = QtGui.QLabel(self.centralwidget)
        self.label_31.setObjectName(_fromUtf8("label_31"))
        self.gridLayout_6.addWidget(self.label_31, 0, 1, 1, 1)
        self.bat_lbl = QtGui.QLabel(self.centralwidget)
        self.bat_lbl.setMinimumSize(QtCore.QSize(0, 17))
        self.bat_lbl.setMaximumSize(QtCore.QSize(110, 17))
        self.bat_lbl.setObjectName(_fromUtf8("bat_lbl"))
        self.gridLayout_6.addWidget(self.bat_lbl, 0, 3, 1, 1)
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout_6.addWidget(self.label_5, 0, 0, 1, 1)
        self.bat_lcd2 = QtGui.QLCDNumber(self.centralwidget)
        self.bat_lcd2.setObjectName(_fromUtf8("bat_lcd2"))
        self.gridLayout_6.addWidget(self.bat_lcd2, 1, 2, 1, 1)
        self.label_32 = QtGui.QLabel(self.centralwidget)
        self.label_32.setObjectName(_fromUtf8("label_32"))
        self.gridLayout_6.addWidget(self.label_32, 0, 2, 1, 1)
        self.gridLayout_6.setColumnMinimumWidth(0, 1)
        self.gridLayout_6.setColumnMinimumWidth(1, 1)
        self.gridLayout_6.setColumnMinimumWidth(2, 1)
        self.verticalLayout_5.addLayout(self.gridLayout_6)
        self.verticalLayout_4.addLayout(self.verticalLayout_5)
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setMinimumSize(QtCore.QSize(419, 292))
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.gridLayout_8 = QtGui.QGridLayout(self.frame)
        self.gridLayout_8.setObjectName(_fromUtf8("gridLayout_8"))
        self.poseVizFrame = QtGui.QVBoxLayout()
        self.poseVizFrame.setSizeConstraint(QtGui.QLayout.SetDefaultConstraint)
        self.poseVizFrame.setObjectName(_fromUtf8("poseVizFrame"))
        self.gridLayout_8.addLayout(self.poseVizFrame, 1, 0, 1, 1)
        self.label_4 = QtGui.QLabel(self.frame)
        self.label_4.setMaximumSize(QtCore.QSize(16777215, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout_8.addWidget(self.label_4, 0, 0, 1, 1)
        self.verticalLayout_4.addWidget(self.frame)
        self.horizontalLayout_5.addLayout(self.verticalLayout_4)
        self.horizontalLayout_5.setStretch(0, 1)
        self.horizontalLayout_5.setStretch(1, 1)
        self.verticalLayout_6.addLayout(self.horizontalLayout_5)
        self.gridLayout_4.addLayout(self.verticalLayout_6, 0, 0, 1, 1)
        RoboticsMain.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(RoboticsMain)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1125, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        RoboticsMain.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(RoboticsMain)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        RoboticsMain.setStatusBar(self.statusbar)

        self.retranslateUi(RoboticsMain)
        QtCore.QMetaObject.connectSlotsByName(RoboticsMain)

    def retranslateUi(self, RoboticsMain):
        RoboticsMain.setWindowTitle(QtGui.QApplication.translate("RoboticsMain", "Robotics Diagnostics and Testing", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("RoboticsMain", "<html><head/><body><p><span style=\" font-weight:600;\">Front CV Data</span></p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("RoboticsMain", "<html><head/><body><p><span style=\" font-weight:600;\">Down CV Data</span></p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label_7.setText(QtGui.QApplication.translate("RoboticsMain", "Relative distance", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("RoboticsMain", "Relative angle", None, QtGui.QApplication.UnicodeUTF8))
        self.label_22.setText(QtGui.QApplication.translate("RoboticsMain", "Relative distance", None, QtGui.QApplication.UnicodeUTF8))
        self.label_26.setText(QtGui.QApplication.translate("RoboticsMain", "Relative angle", None, QtGui.QApplication.UnicodeUTF8))
        self.label_8.setText(QtGui.QApplication.translate("RoboticsMain", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.label_21.setText(QtGui.QApplication.translate("RoboticsMain", "Pitch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_24.setText(QtGui.QApplication.translate("RoboticsMain", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.label_28.setText(QtGui.QApplication.translate("RoboticsMain", "Pitch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_9.setText(QtGui.QApplication.translate("RoboticsMain", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.label_20.setText(QtGui.QApplication.translate("RoboticsMain", "Yaw", None, QtGui.QApplication.UnicodeUTF8))
        self.label_25.setText(QtGui.QApplication.translate("RoboticsMain", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.label_27.setText(QtGui.QApplication.translate("RoboticsMain", "Yaw", None, QtGui.QApplication.UnicodeUTF8))
        self.label_10.setText(QtGui.QApplication.translate("RoboticsMain", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("RoboticsMain", "<html><head/><body><p>Obj. Type</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label_23.setText(QtGui.QApplication.translate("RoboticsMain", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.label_29.setText(QtGui.QApplication.translate("RoboticsMain", "<html><head/><body><p>Obj. Type</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label_11.setText(QtGui.QApplication.translate("RoboticsMain", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.label_12.setText(QtGui.QApplication.translate("RoboticsMain", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.label_13.setText(QtGui.QApplication.translate("RoboticsMain", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.label_17.setText(QtGui.QApplication.translate("RoboticsMain", "Linear", None, QtGui.QApplication.UnicodeUTF8))
        self.label_14.setText(QtGui.QApplication.translate("RoboticsMain", "Roll", None, QtGui.QApplication.UnicodeUTF8))
        self.label_15.setText(QtGui.QApplication.translate("RoboticsMain", "Pitch", None, QtGui.QApplication.UnicodeUTF8))
        self.label_16.setText(QtGui.QApplication.translate("RoboticsMain", "Yaw", None, QtGui.QApplication.UnicodeUTF8))
        self.label_18.setText(QtGui.QApplication.translate("RoboticsMain", "Angular", None, QtGui.QApplication.UnicodeUTF8))
        self.label_19.setText(QtGui.QApplication.translate("RoboticsMain", "Select Controller", None, QtGui.QApplication.UnicodeUTF8))
        self.keyboardControl.setText(QtGui.QApplication.translate("RoboticsMain", "Keyboard", None, QtGui.QApplication.UnicodeUTF8))
        self.manualControl.setText(QtGui.QApplication.translate("RoboticsMain", "PS3", None, QtGui.QApplication.UnicodeUTF8))
        self.autonomousControl.setText(QtGui.QApplication.translate("RoboticsMain", "Autonomus", None, QtGui.QApplication.UnicodeUTF8))
        self.attemptPS3.setText(QtGui.QApplication.translate("RoboticsMain", "Change", None, QtGui.QApplication.UnicodeUTF8))
        self.preLeft.setText(QtGui.QApplication.translate("RoboticsMain", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.preRight.setText(QtGui.QApplication.translate("RoboticsMain", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.postLeft.setText(QtGui.QApplication.translate("RoboticsMain", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.preBottom.setText(QtGui.QApplication.translate("RoboticsMain", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.postRight.setText(QtGui.QApplication.translate("RoboticsMain", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.posBottom.setText(QtGui.QApplication.translate("RoboticsMain", "TextLabel", None, QtGui.QApplication.UnicodeUTF8))
        self.label_30.setText(QtGui.QApplication.translate("RoboticsMain", "Planner Info", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setToolTip(QtGui.QApplication.translate("RoboticsMain", "<html><head/><body><p>Approximate resolution to set the size of the video displays</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setItemText(0, QtGui.QApplication.translate("RoboticsMain", "1024*768", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setItemText(1, QtGui.QApplication.translate("RoboticsMain", "1280*720", None, QtGui.QApplication.UnicodeUTF8))
        self.resSelect.setItemText(2, QtGui.QApplication.translate("RoboticsMain", "1920x1080", None, QtGui.QApplication.UnicodeUTF8))
        self.label_31.setText(QtGui.QApplication.translate("RoboticsMain", "Temperature", None, QtGui.QApplication.UnicodeUTF8))
        self.bat_lbl.setText(QtGui.QApplication.translate("RoboticsMain", "Battery 1", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("RoboticsMain", "Approximate resolution", None, QtGui.QApplication.UnicodeUTF8))
        self.label_32.setText(QtGui.QApplication.translate("RoboticsMain", "Battery 2", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("RoboticsMain", "IMU Pose", None, QtGui.QApplication.UnicodeUTF8))

from pyqtgraph import GraphicsLayoutWidget
