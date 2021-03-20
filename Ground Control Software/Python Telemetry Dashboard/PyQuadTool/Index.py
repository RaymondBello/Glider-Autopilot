#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Toolbox for modeling and control of quadcopter air robots.

This Toolbox in Python language allows modeling and control of square-type stereo robots, in a three-dimensional environment in which physical characteristics of the environment are modeled as well as the plant (robot).In the same way, the development of controllers that can be validated on the software from simulations and graphs is raised.Trajectory tracking will also be taken into account for the navigation of the robot.

Version 1.2 - 3D interface, graphs of status variables, trajectory interpolator, navigation control.
"""
from __future__ import print_function

import pyqtgraph.opengl as gl

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import numpy as np
from quad import*
from control import*
from Model import*
from PID_log import*
from data_log import*

from pyqtgraph.dockarea import *


app = QtGui.QApplication([])
win = QtGui.QMainWindow()
area = DockArea()
win.setCentralWidget(area)
# win.showMaximized()
win.resize(1500, 800)
win.setWindowTitle('PyQuadTool: Toolbox modelado y control Quadcopter')

## Create docks, place them into the window one at a time.
## Note that size arguments are only a suggestion; docks will still have to
## fill the entire dock area and obey the limits of their internal widgets.
d1 = Dock("Dock1", size=(1, 1),closable=True)     ## give this dock the minimum possible size
d2 = Dock("Dock2 - Console", size=(500,150), closable=True)
d3 = Dock("Dock3", size=(500,400))
d4 = Dock("Grafica 1 (X)", size=(500,200))
d5 = Dock("Grafica 2 (Y)", size=(500,200))
d6 = Dock("Grafica 3 (Z)", size=(500,200))
d7 = Dock("Grafica 4 (Yaw)", size=(500,200))
d8 = Dock("Grafica 5 (Pitch)", size=(500,200))
d9 = Dock("Grafica 6 (Roll)", size=(500,200))
d10 = Dock("Grafica 7 (Velocidad)", size=(500,200))
d11 = Dock("Grafica 8 (Rotores)", size=(500,200))
#d12 = Dock("Grafica 9 (Rotores)", size=(500,200))
area.addDock(d1, 'left')      ## place d1 at left edge of dock area (it will fill the whole space since there are no other docks yet)
area.addDock(d2, 'right')     ## place d2 at right edge of dock area
area.addDock(d3, 'bottom', d1)## place d3 at bottom edge of d1
area.addDock(d4, 'top', d2)     ## place d4 at right edge of dock area
area.addDock(d5, 'below', d4)  
area.addDock(d6, 'below', d5)
area.addDock(d7, 'top', d2)     ## place d7 at right edge of dock area
area.addDock(d8, 'below', d7)  
area.addDock(d9, 'below', d8)
area.addDock(d10, 'top', d2)     ## place d7 at right edge of dock area
#area.addDock(d12, 'below', d10)     ## place d7 at right edge of dock area
area.addDock(d11, 'below', d10)
## Test ability to move docks programatically after they have been placed
#area.moveDock(d4, 'top', d2)     ## move d4 to top edge of d2
#area.moveDock(d6, 'above', d4)   ## move d6 to stack on top of d4
#area.moveDock(d5, 'top', d2)     ## move d5 to top edge of d2


## Add widgets into each dock

## first dock gets save/restore buttons
w1 = pg.LayoutWidget()
label = QtGui.QLabel(""" -- PyQuadTool... Toolbox modeled and control quadcopter
This Toolbox in Python Language allows modeling and control of quadricopter-type air robots, in an environment
 Three-dimensional in which physical characteristics of the environment are modeled as well as the plant (robot).The same way
 The software has a control architecture that can be validated about the software from
 Simulations and graphics.Trajectory tracking will also be taken into account for the navigation of the
robot.

Version 1.2 -
3D Interface, Graphics of State Variables, Trajectory Interpolator, Navigation Control.
Juan Jose Martinez Moritz (2015) Pontificia Universidad Javeriana
""")
saveBtn = QtGui.QPushButton('Save Order of the Graphics')
restoreBtn = QtGui.QPushButton('Restore Order of graphics')
restoreBtn.setEnabled(False)
w1.addWidget(label, row=0, col=0)
w1.addWidget(saveBtn, row=1, col=0)
w1.addWidget(restoreBtn, row=2, col=0)
# d1.addWidget(w1)
state = None

def save():
	global state
	state = area.saveState()
	restoreBtn.setEnabled(True)
def load():
	global state
	area.restoreState(state)
saveBtn.clicked.connect(save)
restoreBtn.clicked.connect(load)


w2 = pg.console.ConsoleWidget()
d2.addWidget(w2)

## Hide title bar on dock 3
d3.hideTitleBar()


w = gl.GLViewWidget()

#w.setBackgroundColor('b')
w.show()
w.setWindowTitle('Visualization of the quadrotor')
w.setCameraPosition(distance=8, azimuth=-45, elevation = 30)


g = gl.GLGridItem()
g.scale(3,3,1)
w.addItem(g)
h0 = 0

model = Model("glider.obj")
md = gl.MeshData(vertexes=model.returnMesh())
quadr = gl.GLMeshItem(meshdata=md, color=(0.7, 0.7, 1, 1), smooth=False, shader='shaded', glOptions='opaque')
quadr.translate(0, 0, h0)

arriba = False

ardrone = quad()

ts = ardrone.ts

clog = Control_log(ts)

tm = ts


d3.addWidget(w)

#Graficas-------------------------------------------------------------------------------------------------


w4 = pg.PlotWidget(title="Displacement (x)")

# Use automatic downsampling and clipping to reduce the drawing load
w4.setDownsampling(mode='peak')
w4.setClipToView(True)
curvex = w4.plot()
curvexd = w4.plot(pen = (255,0,0))
w4.setLabel('bottom', 'Time', 's')
w4.setLabel('left', 'X', 'm')
w4.addLegend()
l4 = w4.plotItem.legend
l4.addItem(curvex, "X drone")
l4.addItem(curvexd, "X trajectory")

d4.addWidget(w4)

w5 = pg.PlotWidget(title="Displacement (Y)")

# Use automatic downsampling and clipping to reduce the drawing load
w5.setDownsampling(mode='peak')
w5.setClipToView(True)
curvey = w5.plot()
curveyd = w5.plot(pen = (255,0,0))
w5.setLabel('bottom', 'Time', 's')
w5.setLabel('left', 'Y', 'm')
w5.addLegend()
l5 = w5.plotItem.legend
l5.addItem(curvey, "Y drone")
l5.addItem(curveyd, "Y Trajectory")

d5.addWidget(w5)

w6 = pg.PlotWidget(title="Height (Z)")

# Use automatic downsampling and clipping to reduce the drawing load
w6.setDownsampling(mode='peak')
w6.setClipToView(True)
curvez = w6.plot(name='Z drone')
curvezd = w6.plot(pen = (255,0,0), name='Z Trajectory')
w6.setLabel('bottom', 'Time', 's')
w6.setLabel('left', 'Z', 'm')
w6.addLegend()
l6 = w6.plotItem.legend
l6.addItem(curvez, "Z drone")
l6.addItem(curvezd, "Z Trajectory")

d6.addWidget(w6)

w7 = pg.PlotWidget(title="Rotacion (Yaw)")

# Use automatic downsampling and clipping to reduce the drawing load
w7.setDownsampling(mode='peak')
w7.setClipToView(True)
curveyaw = w7.plot()
w7.setLabel('bottom', 'Time', 's')
w7.setLabel('left', 'Yaw', 'rad')

d7.addWidget(w7)

w8 = pg.PlotWidget(title="Rotacion (Pitch)")

# Use automatic downsampling and clipping to reduce the drawing load
w8.setDownsampling(mode='peak')
w8.setClipToView(True)
curvepitch = w8.plot()
w8.setLabel('bottom', 'Time', 's')
w8.setLabel('left', 'Pitch', 'rad')

d8.addWidget(w8)


w9 = pg.PlotWidget(title="Rotacion (Roll)")
# Use automatic downsampling and clipping to reduce the drawing load
w9.setDownsampling(mode='peak')
w9.setClipToView(True)
curveroll = w9.plot()
w9.setLabel('bottom', 'Time', 's')
w9.setLabel('left', 'Roll', 'rad')

d9.addWidget(w9)


w10 = pg.PlotWidget(title="Speed")

# Use automatic downsampling and clipping to reduce the drawing load
w10.setDownsampling(mode='peak')
w10.setClipToView(True)
curvespeed = w10.plot()
w10.setLabel('bottom', 'Time', 's')
w10.setLabel('left', 'Speed', 'm/s')

d10.addWidget(w10)


#Velocidades Angulares de los rotores
w11 = pg.PlotWidget(title="w Rotors")

# Use automatic downsampling and clipping to reduce the drawing load
w11.setDownsampling(mode='peak')
w11.setClipToView(True)
curveqd1 = w11.plot(pen = (255,0,0))
curveqd2 = w11.plot(pen = (0,255,0))
curveqd3 = w11.plot(pen = (0,0,255))
curveqd4 = w11.plot(pen = ('y'))
w11.setLabel('bottom', 'Time', 's')
w11.setLabel('left', 'Angular velocity', 'rad/s')

d11.addWidget(w11)


####  Load the trajectory. ----------------------------------------------------------------------------

from gentraj import *

gen = gentraj(3)
traj = gen.traj
npt = traj.Time.size


dtrajYaw = np.zeros((npt))
trajYaw = np.zeros((npt))
trajYaw[1:npt]=np.arctan2(traj.Y[1:npt]-traj.Y[0:npt-1],traj.X[1:npt]-traj.X[0:npt-1])

Yaw = np.unwrap(trajYaw)

# Visualization of the Trayectoria
pts = np.vstack([traj.X,traj.Y,traj.Z]).transpose()
plt = gl.GLLinePlotItem(pos=pts,color=pg.glColor(('r')), width=1,antialias=True)
w.addItem(plt)

pt2 = ([ardrone.X[3], ardrone.X[4], ardrone.X[5]])
pl2 = gl.GLLinePlotItem(pos=pt2,color=pg.glColor( (0, 255, 255)),width=1, antialias=True)
w.addItem(pl2)

w.addItem(quadr)
n = 0
nn = 0

timestep=50

data = data_log()



def QuadSym():
	global quadr, ardrone, arriba, ptr, n, nn, pl2
	#Para el 3D
	
	if(ardrone.time <= 30):

	   #Graficar

		global data, timestep

		data.update_data(ardrone,traj.X[nn], traj.Y[nn],-traj.Z[nn])
		
#                            X           Y           Z     yaw
		control(traj.X[nn], traj.Y[nn],-traj.Z[nn], 0, ardrone,clog)
		
		if(n==10):
			nn = nn + 1
			n = 0
		n = n +1
		
			
			
		if timestep>=50:

			data.set_curves(curvex,curvey,curvez,curvexd,curveyd,curvezd,curveyaw,curveroll,curvepitch,curveqd1,curveqd2,curveqd3,curveqd4,curvespeed)
			timestep=-1

			quadr.resetTransform()
			quadr.translate(ardrone.X[3], ardrone.X[4], -ardrone.X[5],local=False)
			quadr.rotate(np.degrees(ardrone.X[0]),1,0,0,local=True)
			quadr.rotate(np.degrees(ardrone.X[1]),0,1,0,local=True)
			quadr.rotate(np.degrees(ardrone.X[2]),0,0,1,local=True)
			
			pt2 = data.quad_traj()
			pl2.setData(pos=pt2,color=pg.glColor((255,255,255)), antialias=True)

		timestep=timestep+1

	else:
		quadr.translate(0,0,0)

# timer = QtCore.QTimer()
# timer.timeout.connect(QuadSym)
# timer.start(ts*1000)
# timer.timeout.connect(lambda: None)


win.show()



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
	import sys
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()

