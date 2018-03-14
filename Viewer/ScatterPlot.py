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
        with open(self.sim_file) as s_f:
            data = json.load(s_f)
        self.sensors = data["Agents"]["sensors"]
        self.x_subs = 0
        self.y_subs = 0

        self.scatterData = {}  # Dictionary [Index][[ScatterPlotData],[plotCurve],[curvePoint]]
        self.scatterPlot = pg.PlotWidget(title="Prodifcon Viewer")
        self.PayoffPlot = pg.PlotWidget(title = "Single benefit ")
        self.benefitPlot = pg.PlotWidget(title="Team Benefit")  #plot widget added for benefit
        self.potentialPlot = pg.PlotWidget(title="Potential of Game")    #plot widget added for potential of game 
        self.coveredsquarePlot = pg.PlotWidget(title="Squares Covered") #plot widget for number of viewed squared
        self.fovData = {}
        self.thiefData = {}
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
        self.x_subs = data['Area']['length'] / data['Area']['cols']
        self.y_subs = data['Area']['height'] / data['Area']['rows']
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
        text = pg.TextItem(html='<div style="text-align: font-size: 12pt;">MS</span></div>', border='y', fill=(0, 255, 0))
        text.setPos(center_xy[0]-40, center_xy[1]-40)
        self.scatterPlot.addItem(text)

        
    @QtCore.Slot(float, float, object)  
    def updateScatterData(self, x, y, name): # Connected with data_ready ZmqThread - Manages the plot curves
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        self.semaphore.acquire()
        if name not in self.scatterData:
            # Soluzione temporanea per la selezione del colore
            scatter = self.scatterPlot.plot(x=[x], y=[y], pen=colors[int(name.split('_')[1]) % len(colors)], symbol='+', symbolSize=5, pxMode=True)
            self.scatterData[name] = [ScatterPlotData(10), scatter]
            cPoint = pg.CurvePoint(scatter)
            self.scatterData[name].append(cPoint)   # Construction of curve point
            self.scatterPlot.addItem(cPoint)        # Add to scatterPlot
            label = pg.TextItem(name, anchor=(0.5, 0))
            label.setParentItem(self.scatterData[name][2])
        self.scatterData[name][0].add_data(x, y)
        self.semaphore.release()

    @QtCore.Slot(int, float, float)
    def updateThiefRect(self, id, x, y):
        if id not in self.thiefData:
            self.thiefData[id] = {}
            self.thiefData[id]['rect'] = QG.QGraphicsRectItem()
            self.scatterPlot.addItem(self.thiefData[id]['rect'])
        self.thiefData[id]['x'] = x
        self.thiefData[id]['y'] = y

    @QtCore.Slot(int, object, object, object)
    def updateFovData(self, id, x, y, heading):
        self.semaphore.acquire()
        if id not in self.fovData:
            self.fovData[id] = {}
            #self.fovData[id]['shape'] = QG.QGraphicsPathItem()
            #self.fovData[id]['shape'].setPen(pg.mkPen('y'))
            self.fovData[id]['ellipse'] = QG.QGraphicsEllipseItem()
            self.fovData[id]['radius'] = self.sensors[id][0]
            self.scatterPlot.addItem(self.fovData[id]['ellipse'])
            #self.fovData[id]['ellipse'].setBrush(pg.mkBrush('b'))

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
                radius = self.fovData[i]['radius']
                self.fovData[i]['ellipse'].setRect(QtCore.QRectF(X-radius/2, Y-radius/2, radius, radius))
                self.fovData[i]['ellipse'].setStartAngle((np.rad2deg(self.fovData[i]['h'])+ 90  - 90/2 +180)*16)
                self.fovData[i]['ellipse'].setSpanAngle(360*4)

                radialGrad = QG.QRadialGradient(QtCore.QPointF(X, Y), radius/2)
                b = QG.QColor(QtCore.Qt.black)
                b.setAlpha(255)
                bb = QG.QColor(51, 0, 0)
                bb.setAlpha(255)
                r = QG.QColor(QtCore.Qt.red)
                r.setAlpha(255)
                ar = QG.QColor(255, 153, 51) #arancio
                ar.setAlpha(255)
                bb1 = QG.QColor(204, 0, 0) #dark red
                bb1.setAlpha(125)
                y = QG.QColor(QtCore.Qt.yellow)
                y.setAlpha(125)
                radialGrad.setColorAt(0, b)
                radialGrad.setColorAt(0.125, bb)
                radialGrad.setColorAt(0.250, r)
                radialGrad.setColorAt(0.750, bb1)
                radialGrad.setColorAt(0.500, ar)
                radialGrad.setColorAt(0.950, y)
                radialGrad.setColorAt(0.250, r)
                radialGrad.setColorAt(1, y)
                self.fovData[i]['ellipse'].setBrush(radialGrad)

        for i in self.thiefData:
            if 'x' in self.thiefData[i]:
                X = self.thiefData[i]['x']
                Y = self.thiefData[i]['y']
                self.thiefData[i]['rect'].setRect(QtCore.QRectF(X-4*self.x_subs, Y-4*self.y_subs, 8*self.x_subs, 8*self.y_subs))
                radialGrad = QG.QRadialGradient(QtCore.QPointF(X, Y), 4*self.x_subs)
                #b = QG.QColor(QtCore.Qt.green)
                #b.setAlpha(128) # TRASOARENCY
                #r = QG.QColor(255, 0, 0)
                #r.setAlpha(128)
                #y = QG.QColor(255, 0, 0)

                l1 = QG.QColor(0, 255, 0)
                l1.setAlpha(255)
                l2 = QG.QColor(0, 225, 25)
                l2.setAlpha(125)
                l3 = QG.QColor(0, 204, 51)
                l3.setAlpha(125)
                l4 = QG.QColor(0, 175, 75)
                l4.setAlpha(255)
                l5 = QG.QColor(0, 153, 102)
                l5.setAlpha(255)
                l6 = QG.QColor(0, 102, 153)
                l6.setAlpha(255)
                l7 = QG.QColor(0, 75, 175)
                l7.setAlpha(255)
                l8 = QG.QColor(0, 51, 204)
                l8.setAlpha(128)
                l9 = QG.QColor(0, 25, 225)
                l9.setAlpha(128)
                l10 = QG.QColor(0, 0, 255)
                l10.setAlpha(128)
                #y.setAlpha(128)
                radialGrad.setColorAt(0.01, l1)
                radialGrad.setColorAt(0.1, l2)
                radialGrad.setColorAt(0.2, l3)
                radialGrad.setColorAt(0.3, l4)
                radialGrad.setColorAt(0.4, l5)
                radialGrad.setColorAt(0.5, l6)
                radialGrad.setColorAt(0.6, l7)
                radialGrad.setColorAt(0.7, l8)
                radialGrad.setColorAt(0.8, l9)
                radialGrad.setColorAt(1, l10)
                #radialGrad.setColorAt(0, b)
                #radialGrad.setColorAt(0.5, r)
                #radialGrad.setColorAt(1, y)
                self.thiefData[i]['rect'].setBrush(radialGrad)

        if self.benefitValue.shape[0] > 0:  
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
     