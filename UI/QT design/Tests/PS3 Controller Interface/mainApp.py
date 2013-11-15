from UIFeedback import *
from PyQt4 import QtCore, QtGui
import sys
import sixaxis


class MyForm(QtGui.QMainWindow):
    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

       # QtCore.QObject.connect(self.ui.pushButton, QtCore.SIGNAL('clicked()'), self.addLine)
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
    def addLine(self):
        self.ui.textEdit.append(self.ui.lineEdit.text())



if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = MyForm()
    AppWindow.show()
    sys.exit(app.exec_())

    sixaxis.init("/dev/input/js0")
    while(True):
        state = sixaxis.get_state()
        if state['triangle']:
            print("_______________TRIANGLE PRESSED _____________")
        if state('square'):
            print("++++++++++++++SQUARE++++++++++++++++")

