from pyqtgraph.Qt import QtGui, QtCore
import numpy
import pyqtgraph

#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pyqtgraph.GraphicsWindow(title="Basic plotting examples")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')

p2 = win.addPlot(title="allo")
p2.plot((1,2,3,4,1,1,1,1,1), pen=(255,0,0))
p2.plot(numpy.random.normal(size=100)+5, pen=(0,255,0))

win.nextRow()

p6 = win.addPlot(title="Updating plot")
lower_updating = p6.plot(pen='b')
data = numpy.random.normal(size=(100))  # returns 1000 set of 10 digits as a numpy ndarray
ptr = 0
def update():
    global lower_updating, data, ptr, p6
    data[0].add(1)
    curve.setData(data[0])
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
        ptr += 1

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)


win.nextRow()



"""
x2 = np.linspace(-100, 100, 1000)
data2 = np.sin(x2) / x2
p8 = win.addPlot(title="Region Selection")
p8.plot(data2, pen=(255,255,255,200))
lr = pg.LinearRegionItem([400,700])
lr.setZValue(-10)
p8.addItem(lr)


p9 = win.addPlot(title="Zoom on selected region")
p9.plot(data2)

def updatePlot():
    p9.setXRange(*lr.getRegion(), padding=0)

def updateRegion():
    lr.setRegion(p9.getViewBox().viewRange()[0])
lr.sigRegionChanged.connect(updatePlot)
p9.sigXRangeChanged.connect(updateRegion)
updatePlot()
"""
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
