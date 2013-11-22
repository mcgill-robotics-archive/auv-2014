from ui import *
import numpy
import sys


class Main(QtGui.QMainWindow):

    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.timer = QtCore.QTimer()

#   MULTIPLE STATIC CURVES IN THE SAME PLOT
#        #FILL THE PLOT
        p2 = self.ui.graphicsView.addPlot(title="Multiple curves")
        p2.plot(numpy.random.normal(size=100), pen=(255,0,0))
        p2.plot(numpy.random.normal(size=100)+5, pen=(0,255,0))
        p2.plot(numpy.random.normal(size=100)+10, pen=(0,0,255))

        self.ui.graphicsView.nextRow()
#   SINGLE UPDATING CURVE
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

        global lower_updating, data, p6, second_updating, data2
        p6 = self.ui.graphicsView.addPlot(title="Updating plot")
        lower_updating = p6.plot()
        second_updating = p6.plot(pen="y")
        data = []
        data2 =[]
        for i in range(0, 100, 01):
            data.append(i)
            data2.append(i+400)

    def update(self):
        global lower_updating, data, data2, p6, second_updating
     #   data.remove(data[0])
        data.append(data[len(data)-1]%3.14*31)
        data2.append(data2[len(data2)-1]%3.14*31)
        lower_updating.setData(data)
        second_updating.setData(data2)
        p6.setXRange(len(data)-100, len(data))
       # p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = Main()
    AppWindow.show()
    sys.exit(app.exec_())