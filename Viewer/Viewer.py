"""
    Modulo di visualizzazione per il sonar attivo.
    Si comporta come un applicativo Server.
"""
import sys
import logging
from PySide import QtCore
from pyqtgraph.Qt import QtGui
import ScatterPlot
import ZmqThread  
import pyqtgraph as pg
import numpy as np

 
if __name__ == "__main__":
    try:
        app = QtGui.QApplication([])
        th = ZmqThread.ZmqThread(sys.argv[1])
        area_data = th.initStage()  
        viewer = ScatterPlot.Viewer(area_data[1], area_data[0], sys.argv[1])
        th.data_ready.connect(viewer.updateScatterData) 
        th.fov_ready.connect(viewer.updateFovData)
        th.data_benefit.connect(viewer.updatebenefit)
        th.data_potential.connect(viewer.updatePotential)
        th.data_coveredsquare.connect(viewer.updatecoveredSquare)
        th.thief_ready.connect(viewer.updateThiefRect)
        th.start()
        # layout.addWidget(viewer, 1, 1) # plot goes on right side, spanning 3 rows


        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            logging.info("Lauching the Qt viewer...")
            QtGui.QApplication.instance().exec_()
        th.stop()
        exit(0)

    except KeyboardInterrupt:
        print "Keyboard Interrupt"