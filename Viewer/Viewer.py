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

if __name__ == "__main__":
    try:
        app = QtGui.QApplication([])

        th = ZmqThread.ZmqThread(sys.argv[1])
        area_data = th.initStage()  # This runs the sonar map building method
        
        viewer = ScatterPlot.Viewer(area_data[1], area_data[0])
        th.data_ready.connect(viewer.updateScatterData)
        th.fov_ready.connect(viewer.updateFovData)
        th.start()

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            logging.info("Lauching the Qt viewer...")
            QtGui.QApplication.instance().exec_()
        th.stop()

    except KeyboardInterrupt:
        print "Keyboard Interrupt"