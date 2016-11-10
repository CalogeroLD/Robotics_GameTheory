import numpy as np
from PySide import QtCore
from PySide import QtGui as QG
import PySide
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import threading
import json

max_plot_dim = 1000 #benefit che vedo sono relativi agli ultimi 100 step

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
    def __init__(self, x_lim, y_lim, sim_file):
        super(Viewer, self).__init__()
        self.sim_file = sim_file

        self.scatterData = {}  # Dictionary [Index][[ScatterPlotData],[plotCurve],[curvePoint]]
        self.scatterPlot = pg.PlotWidget(title="Prodifcon Viewer")
        self.PayoffPlot = pg.PlotWidget(title = "Single benefit ")
        self.benefitPlot = pg.PlotWidget(title="Team Benefit")  #plot widget added for benefit
        self.potentialPlot = pg.PlotWidget(title="Potential of Game")    #plot widget added for potential of game 
        self.coveredsquarePlot = pg.PlotWidget(title="Squares Covered") #plot widget for number of viewed squared
        self.fovData = {}
        self.initUI(x_lim, y_lim)
        self.semaphore = threading.Lock()
        self.timer = QtCore.QBasicTimer()
        self.timer.start(100, self)
        
        self.benefitValue = np.ndarray((0,0)) #array with stored benefit values
        self.potentialValue = np.ndarray((0,0)) #array with storedo potential data
        self.coveredsquareValue = np.ndarray((0,0))

    def initUI(self, x_lim, y_lim):
        self.resize(600, 600)
        self.setWindowTitle('Dynamic Coverage Viewer')
        self.scatterPlot.setRange(xRange=[0, x_lim], yRange=[0, y_lim])
        self.scatterPlot.setLabel('left', "North", units='m')
        self.scatterPlot.setLabel('bottom', "East", units='m')
        self.scatterPlot.showGrid(x=True, y=True)

        self.benefitPlot.showGrid(x=True, y=True)
        self.benefitPlot.enableAutoRange()
        self.benefitPlot.setLabel('bottom', "time step")
        self.benefit_p = self.benefitPlot.plot(y=[0], pen = 'r')

        self.potentialPlot.showGrid(x=True, y=True)
        self.potentialPlot.enableAutoRange()
        self.potentialPlot.setLabel('bottom', "time step")
        self.potential_p = self.potentialPlot.plot(y=[0])

        self.coveredsquarePlot.showGrid(x=True, y=True)
        self.coveredsquarePlot.enableAutoRange()
        self.coveredsquarePlot.setLabel('bottom', "time step")
        self.coveredsquare_p = self.coveredsquarePlot.plot(y=[0], pen = 'g')

        self.PayoffPlot.showGrid(x=True, y=True)
        self.PayoffPlot.enableAutoRange()
        self.PayoffPlot.setLabel('bottom', "time step")
        self.Payoff_p = self.PayoffPlot.plot(y=[0], pen = 'g')

        pg.setConfigOptions(antialias=True)

        # setting of layout od viwer
        layout = QtGui.QGridLayout()
        self.setLayout(layout)
        layout.addWidget(self.scatterPlot, 0, 0, 3, 1)
        layout.addWidget(self.benefitPlot, 0, 1)
        layout.addWidget(self.potentialPlot, 1, 1)
        layout.addWidget(self.coveredsquarePlot, 2, 1)
        self.show()
        self.draw_mothership()


    def draw_mothership(self):
        # radar of mother-ship
        with open(self.sim_file) as s_f:
            data = json.load(s_f)
        center = data['Area']['ship']['coord']
        dims = ((data['Area']['length'] / data['Area']['cols'])*data['Area']['ship']['dim_col'], (data['Area']['length'] / data['Area']['rows'])*data['Area']['ship']['dim_row'])
        center_xy = ( (data['Area']['length'] / data['Area']['cols'])*center[0], (data['Area']['height'] / data['Area']['rows'])*center[1])

        r2 = pg.QtGui.QGraphicsRectItem(pg.QtCore.QRectF(center_xy[0] -dims[0]/2, center_xy[1] - dims[1]/2, dims[0], dims[1]))
        r2.setPen(pg.mkPen((0, 0, 0, 100)))
        r2.setBrush(pg.mkBrush('b'))
        self.scatterPlot.addItem(r2)
        # mothership
        r1 = pg.QtGui.QGraphicsRectItem(pg.QtCore.QRectF(center_xy[0] -50, center_xy[1] - 5, 100, 10))
        r1.setPen(pg.mkPen((0, 0, 0, 100)))
        r1.setBrush(pg.mkBrush('r'))
        self.scatterPlot.addItem(r1)

        # Create text object, use HTML tags to specify color/size
        text = pg.TextItem(html='<div style="text-align: font-size: 12pt;">MOTHERSHIP</span></div>', border='y', fill=(0, 255, 0))
        self.scatterPlot.addItem(text)
        text.setPos(center_xy[0], center_xy[1])

    @QtCore.Slot(float, float, object)  #definition of Slot referring to a Signal
    def updateScatterData(self, x, y, name):
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        self.semaphore.acquire()

        if name not in self.scatterData:
            # Soluzione temporanea per la selezione del colore
            scatter = self.scatterPlot.plot(x=[x], y=[y], pen=colors[int(name.split('_')[1]) % len(colors)], symbol='+', symbolSize=5, pxMode=True)
            self.scatterData[name] = [ScatterPlotData(10), scatter]
            cPoint = pg.CurvePoint(scatter)
            self.scatterData[name].append(cPoint)  # Construction of curve point
            self.scatterPlot.addItem(cPoint)  # Add to scatterPlot
            label = pg.TextItem(name, anchor=(0.5, 0))
            label.setParentItem(self.scatterData[name][2])
        self.scatterData[name][0].add_data(x, y)
        self.semaphore.release()

    @QtCore.Slot(int, object, object, object)
    def updateFovData(self, id, x, y, heading):
        self.semaphore.acquire()
        if id not in self.fovData:
            self.fovData[id] = {}
            #self.fovData[id]['shape'] = QG.QGraphicsPathItem()
            #self.fovData[id]['shape'].setPen(pg.mkPen('y'))
            self.fovData[id]['ellipse'] = QG.QGraphicsEllipseItem()
            self.scatterPlot.addItem(self.fovData[id]['ellipse'])
            self.fovData[id]['ellipse'].setBrush(pg.mkBrush('b'))

        self.fovData[id]['x'] = x
        self.fovData[id]['y'] = y
        self.fovData[id]['h'] = heading
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

                self.fovData[i]['ellipse'].setRect(QtCore.QRectF(X-250,Y-250,500,500))
                self.fovData[i]['ellipse'].setStartAngle((np.rad2deg(self.fovData[i]['h'])+ 90  - 90/2 +180)*16)
                self.fovData[i]['ellipse'].setSpanAngle(360*4)

                radialGrad = QG.QRadialGradient(QtCore.QPointF(X, Y), 250)
                radialGrad.setColorAt(0, QtCore.Qt.red)
                radialGrad.setColorAt(0.5, QtCore.Qt.blue)
                radialGrad.setColorAt(1, QtCore.Qt.green)
                self.fovData[i]['ellipse'].setBrush(radialGrad)

        if self.benefitValue.shape[0] > 0:  #se arrivano dati
            self.benefit_p.setData(np.arange(self.benefitValue.shape[0]), self.benefitValue) # setta gli assi del plot
        if self.potentialValue.shape[0] > 0:
            self.potential_p.setData(np.arange(self.potentialValue.shape[0]), self.potentialValue) #arange returns a ndarray
        if self.coveredsquareValue.shape[0] > 0:
            self.coveredsquare_p.setData(np.arange(self.coveredsquareValue.shape[0]), self.coveredsquareValue)
        self.semaphore.release()

    QtCore.Slot(float)
    def updatebenefit(self, benefit):
        self.semaphore.acquire()
        #print self.benefitValue.shape[0], # dim del range dati
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

    QtCore.Slot(float)
    def updatecoveredSquare(self, coveredsquare):
        self.semaphore.acquire()
        if self.coveredsquareValue.shape[0] > max_plot_dim:
            self.coveredsquareValue = np.roll(self.coveredsquareValue, -1)
            self.coveredsquareValue[-1] = coveredsquare
        else:
            self.coveredsquareValue = np.append(self.coveredsquareValue, coveredsquare)
        self.semaphore.release()
     