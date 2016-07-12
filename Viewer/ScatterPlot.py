import numpy as np
from PySide import QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import threading

class ScatterPlotData:
    def __init__(self, max_dim):
        self.x_data = np.ndarray((0, 0))
        self.y_data = np.ndarray((0, 0))
        self.max_dim = max_dim

    def add_data(self, x, y):
        if self.x_data.shape[0] >= self.max_dim:
            self.x_data = np.roll(self.x_data, -1)
            self.x_data[-1] = x
            self.y_data = np.roll(self.y_data, -1)
            self.y_data[-1] = y
        else:
            self.x_data = np.append(self.x_data, x)
            self.y_data = np.append(self.y_data, y)


class Viewer(QtGui.QWidget):
    def __init__(self, x_lim, y_lim):
        super(Viewer, self).__init__()
        self.scatterData = {}  # Dictionary [Index][[ScatterPlotData],[plotCurve],[curvePoint]]
        self.scatterPlot = pg.PlotWidget(title="Prodifcon Viewer")
        self.fovData = {}
        self.initUI(x_lim, y_lim)
        self.semaphore = threading.Lock()
        self.timer = QtCore.QBasicTimer()

        self.timer
        self.timer.start(100, self)

    def initUI(self, x_lim, y_lim):
        self.resize(600, 600)
        self.setWindowTitle('Dynamic Coverage Viewer')

        self.scatterPlot.setRange(xRange=[0, x_lim], yRange=[0, y_lim])
        self.scatterPlot.setLabel('left', "North", units='m')
        self.scatterPlot.setLabel('bottom', "East", units='m')
        self.scatterPlot.showGrid(x=True, y=True)
        pg.setConfigOptions(antialias=True)


        layout = QtGui.QGridLayout()
        self.setLayout(layout)
        layout.addWidget(self.scatterPlot, 0, 0)
        self.show()

    @QtCore.Slot(float, float, object)
    def updateScatterData(self, x, y, name):
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        self.semaphore.acquire()
        print name
        if name not in self.scatterData:
            # Soluzione temporanea per la selezione del colore
            scatter = self.scatterPlot.plot(x=[x], y=[y], pen=colors[int(name.split('_')[1]) % len(colors)], symbol='+', symbolSize=5, pxMode=True)
            self.scatterData[name] = [ScatterPlotData(50), scatter]
            cPoint = pg.CurvePoint(scatter)
            self.scatterData[name].append(cPoint)  # Construction of curve point
            self.scatterPlot.addItem(cPoint)  # Add to scatterPlot
            label = pg.TextItem(name, anchor=(0.5, 0))
            label.setParentItem(self.scatterData[name][2])
        self.scatterData[name][0].add_data(x, y)
        self.semaphore.release()

    @QtCore.Slot(object, object)
    def updateFovData(self, id, x, y):
        self.semaphore.acquire()
        if id not in self.fovData:
            self.fovData[id] = {}
            self.fovData[id]['plot'] = self.scatterPlot.plot(pen='y')  #colore del sensore
        else:
            self.fovData[id]['x'] = x
            self.fovData[id]['y'] = y
        self.semaphore.release()

    def timerEvent(self, *args, **kwargs):
        self.semaphore.acquire()
        for e in self.scatterData:
            elem = self.scatterData[e]
            if elem[0].x_data.shape[0] > 0:
                elem[1].setData(x=elem[0].x_data, y=elem[0].y_data)
                elem[2].setPos(elem[0].x_data[0])
        for i in self.fovData:
            if 'x' in self.fovData[i]:
                X = self.fovData[i]['x']
                Y = self.fovData[i]['y']
                self.fovData[i]['plot'].setData(x=X, y=Y)
        self.semaphore.release()
