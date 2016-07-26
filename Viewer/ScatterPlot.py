import numpy as np
from PySide import QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import threading

max_plot_dim = 100 #benefit che vedo sono relativi agli ultimi 100 step

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
        self.benefitPlot = pg.PlotWidget(setWindowTitle="Benefit")  #plot widget added for benefit
        self.potentialPlot = pg.PlotWidget(setWindowTitle="Benefit")    #plot widget added for potential of game 
        self.fovData = {}
        self.initUI(x_lim, y_lim)
        self.semaphore = threading.Lock()
        self.timer = QtCore.QBasicTimer()
        self.timer.start(100, self)
        
        self.benefitValue = np.ndarray((0,0)) #array with stored benefit values
        self.potentialValue = np.ndarray((0,0)) #array with storedo potential data
        

    def initUI(self, x_lim, y_lim):
        self.resize(600, 600)
        self.setWindowTitle('Dynamic Coverage Viewer')
        self.scatterPlot.setRange(xRange=[0, x_lim], yRange=[0, y_lim])
        self.scatterPlot.setLabel('left', "North", units='m')
        self.scatterPlot.setLabel('bottom', "East", units='m')
        self.scatterPlot.showGrid(x=True, y=True)

        self.benefitPlot.showGrid(x=True, y=True)
        self.benefitPlot.enableAutoRange()
        self.benefit_p = self.benefitPlot.plot(y=[0])

        self.potentialPlot.showGrid(x=True, y=True)
        self.potentialPlot.enableAutoRange()
        self.potential_p = self.potentialPlot.plot(y=[0])

        pg.setConfigOptions(antialias=True)

        # setting of layout od viwer
        layout = QtGui.QGridLayout()
        self.setLayout(layout)
        layout.addWidget(self.scatterPlot, 0, 0, 2, 1)
        layout.addWidget(self.benefitPlot, 0, 1)
        layout.addWidget(self.potentialPlot, 1, 1)
        self.show()

    @QtCore.Slot(float, float, object)  #definition of Slot referring to a Signal
    def updateScatterData(self, x, y, name):
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        self.semaphore.acquire()
        print name
        if name not in self.scatterData:

            ## Create text object, use HTML tags to specify color/size
            text = pg.TextItem(html='<div style="text-align: font-size: 12pt;">MOTHERSHIP</span></div>', border='y', fill=(0, 0, 255))
            self.scatterPlot.addItem(text)
            text.setPos(30, 30)
            # create a circle too indicate position of mothership
            r = pg.CircleROI(30.0, 30.0)
            self.scatterPlot.addItem(r)

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
    
    # takes data and updates plots
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
        if self.benefitValue.shape[0] > 0:  #se arrivano dati
            self.benefit_p.setData(np.arange(self.benefitValue.shape[0]), self.benefitValue) # setta gli assi del plot
        if self.potentialValue.shape[0] > 0:
            self.potential_p.setData(np.arange(self.potentialValue.shape[0]), self.potentialValue) #arange returns a ndarray
        self.semaphore.release()

    QtCore.Slot(float)
    def updatebenefit(self, benefit):
        self.semaphore.acquire()
        print self.benefitValue.shape[0], ' dim del range dati '
        if self.benefitValue.shape[0] > max_plot_dim: 
            self.benefitValue = np.roll(self.benefitValue, -1)
            self.benefitValue[-1] = benefit
        else:
            self.benefitValue = np.append(self.benefitValue, benefit)
        self.semaphore.release()


    QtCore.Slot(float)
    def updatePotential(self, potential):
        self.semaphore.acquire()
        if self.potentialValue.shape[0] > max_plot_dim:
            self.potentialValue = np.roll(self.potentialValue, -1)
            self.potentialValue[-1] = potential
        else:
            self.potentialValue = np.append(self.potentialValue, potential)
        self.semaphore.release()


        