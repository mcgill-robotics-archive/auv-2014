from multext import *
from PyQt4 import QtCore, QtGui
import sys

class MyForm(QtGui.QMainWindow):
    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        QtCore.QObject.connect(self.ui.pushButton, QtCore.SIGNAL('clicked()'), self.addLine)

    def addLine(self):
        self.ui.textEdit.append(self.ui.lineEdit.text())



if __name__ == "__main__":
        app = QtGui.QApplication(sys.argv)
        AppWindow = MyForm()
        AppWindow.show()
        sys.exit(app.exec_())



