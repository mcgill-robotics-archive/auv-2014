# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QtGui
import pyqtgraph as pyqtgraph
import numpy

pyqtgraph.setConfigOptions(antialias=True)

x = numpy.arange(10)
y = numpy.arange(10) %3
top = numpy.linspace(1.0, 3.0, 10)
bottom = numpy.linspace(2, 0.5, 10)

plot = pyqtgraph.plot()
plot.setWindowTitle('pyqtgraph example: ErrorBarItem')
err = pyqtgraph.ErrorBarItem(x=x, y=y, top=top, bottom=bottom, beam=0.5)
print type(plot)
plot.addItem(err)
plot.plot(x, y, symbol='o', pen={'color': 0.8, 'width': 2})

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
