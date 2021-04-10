# -*- coding: utf-8 -*-

"""
Simple use of DataTreeWidget to display a structure of nested dicts, lists, and arrays
"""

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import time


# for generating a traceback object to display
def some_func1():
    return some_func2()
def some_func2():
    try:
        raise Exception()
    except:
        import sys
        return sys.exc_info()[2]


app = QtGui.QApplication([])
d = {
    'Flight Plan': {
        'Info': 1,
        'Date & Time': f"{time.localtime().tm_year}/{time.localtime().tm_mon}/{time.localtime().tm_mday}  {time.localtime().tm_hour}:{time.localtime().tm_min}:{time.localtime().tm_sec}",
        'Waypoints': [
            {
                'id':0,
                'lat':45.8,
                'lon':34.2,
                'alt':453,
                'head':270,
                'speed':14,
            },
            {
                'id':1,
                'lat':15.8,
                'lon':32.2,
                'alt':253,
                'head':240,
                'speed':10,
            },
            {
                'id':2,
                'lat':55.8,
                'lon':37.2,
                'alt':200,
                'head':200,
                'speed':6,
            }
        ]
    },
    'a function': some_func1,
    'a class': pg.DataTreeWidget,
}

tree = pg.DataTreeWidget(data=d)
tree.show()
tree.setWindowTitle('pyqtgraph example: DataTreeWidget')
tree.resize(600,600)


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()