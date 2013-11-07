from PyQt4 import QtGui

import sys

class MainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
      super(MainWindow, self).__init__(parent)  
      self.dowid()
      self.setCentralWidget(self.thewid) 

    def dowid(self):
      self.thewid = QtGui.QGroupBox()
      vbox = QtGui.QVBoxLayout()
      self.radiobuttons = []
      listOfChoices = 'one two three'.split()
      for i, row in enumerate(listOfChoices):
          radio = QtGui.QRadioButton(row)
          if i == 0:
              radio.setChecked(True)
          self.radiobuttons.append(radio)
          vbox.addWidget(radio)
      self.thewid.setLayout(vbox)

    def examine(self):
      for i, radio in enumerate(self.radiobuttons):
        if radio.isChecked():
            print "radio button num " + str(i) + " is checked"
        else:
            print "radio button num " + str(i) + " is NOT checked"

if __name__ == '__main__':
    app = QtGui.QApplication([])
    mainWin = MainWindow()
    mainWin.show()
    rc = app.exec_()
    mainWin.examine()
