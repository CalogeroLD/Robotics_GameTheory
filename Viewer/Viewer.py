"""
    Modulo di visualizzazione per il sonar attivo.
    Si comporta come un applicativo Server.
"""
import sys
import logging
from PySide import QtCore
from pyqtgraph.Qt import QtGui
import ScatterPlot
import ZmqThread   # sta importando la classe dichiarata
import pyqtgraph as pg
import numpy as np



'''
        w = QtGui.QWidget()
        ## Create some widgets to be placed inside
        #plot = pg.PlotWidget()
        #BENEFIT
        pw = pg.PlotWidget()
        #date = np.arange(8) * (3600*24*356)
        pw.plot(y=[1,6,2,4,3,5,6,8], symbol='o')
        pw.show()
        pw.setWindowTitle('pyqtgraph example: customPlot')
        ## Create a grid layout to manage the widgets size and position
        layout = QtGui.QGridLayout()
        w.setLayout(layout)

         ## Add widgets to the layout in their proper positions
        #layout.addWidget(btn, 0, 0) # button goes in upper-left
        #layout.addWidget(text, 1, 0, 3, 1) # text edit goes in middle-left
        #layout.addWidget(listw, 2, 0, 3, 1) # list widget goes in bottom-left
        layout.addWidget(pw, 1, 0) # plot goes on right side, spanning 3 rows

        ## Display the widget as a new window
        w.show()
'''    
if __name__ == "__main__":
    try:
        app = QtGui.QApplication([])
        th = ZmqThread.ZmqThread(sys.argv[1])
        area_data = th.initStage()  
        viewer = ScatterPlot.Viewer(area_data[1], area_data[0])
        th.data_ready.connect(viewer.updateScatterData) #creates a connection between QtSignal and QtCore
        th.fov_ready.connect(viewer.updateFovData)
        th.data_benefit.connect(viewer.updatebenefit)
        th.start()
        # layout.addWidget(viewer, 1, 1) # plot goes on right side, spanning 3 rows


        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            logging.info("Lauching the Qt viewer...")
            QtGui.QApplication.instance().exec_()
        th.stop()
        exit(0)

    except KeyboardInterrupt:
        print "Keyboard Interrupt"