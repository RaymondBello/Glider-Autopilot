#!/usr/bin/python3
# -*- coding: utf-8 -*-

__author__ = "Ray Bello"
__copyright__ = "Copyright 2020, RayTech"
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Ray"
__email__ = "raymondbello11@gmail.com"
__status__ = "Development"


import sys
import io
from os.path import dirname, realpath, join
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import QPushButton,QWidget, QBoxLayout, QVBoxLayout, QFileDialog, QAction
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyqtgraph.console
import numpy as np
import folium
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from statemachine import StateMachine, State
from pyqtgraph.dockarea import *

class MatplotLibCanvas(FigureCanvas):
    
    def __init__(self, width=10,height=10, dpi=100):
        self.fig = Figure(figsize=(width,height),dpi=dpi)
        FigureCanvas.__init__(self,self.fig)
    
    def plot3D(self, xarray,yarray,zarray):
        self.fig.clear()
        self.axe3D = self.fig.add_subplot(111, projection='3d')
        self.axe3D.plot_trisurf(xarray,yarray,zarray, color='red', alpha=0.6,edgecolor='red',shade=1)
        self.axe3D.plot(xarray,yarray,zarray,'ok')
        self.axe3D.set_xlabel('X')
        self.axe3D.set_ylabel('Y')
        self.axe3D.set_zlabel('Z')
        self.draw()
    
    def plot2D(self, xarray, yarray):
        self.fig.clear()
        self.axe2D = self.fig.add_subplot(111)
        self.axe2D.plot(xarray,yarray,'ok')
        self.axe2D.plot(xarray,yarray,'r-')
        self.axe2D.set_xlabel('X')
        self.axe2D.set_ylabel('Y')
        self.axe2D.set_zlabel('Z')
        self.draw()
        

class FiniteStateMachine(StateMachine):
    ''
    idle = State('Idle')
    active = State('Active', initial=True)
    
    initialize = idle.to(active)
    abort = active.to(idle)

class MainWindow(QWidget):
     
    MainWindowTitle = "UAV Ground Control System"
    __version__ = 0.1
    
    WINDOW_WIDTH = 1500
    WINDOW_HEIGHT = 800
    
    background_color = (36, 37, 41)
    aspect_ratio_locked = True
    
    # Button styles
    ButtonStyle_white = "background-color:rgb(255, 255, 255);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_red = "background-color:rgb(200, 0, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_green = "background-color:rgb(0, 155, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_yellow = "background-color:rgb(255, 255, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    
    # Initial Map coordinates
    coord = (45.40415841728934, -75.70669454423376)
    
    
    def __init__(self):
        self.app = pg.mkQApp()
        self.win = QtGui.QMainWindow()
        self.area = area = DockArea()
        
        self.win.setCentralWidget(area)
        self.win.resize(self.WINDOW_WIDTH, self.WINDOW_HEIGHT)
        self.win.setWindowTitle(self.MainWindowTitle)
        super().__init__()
        
        self.init_FSM()
        
        self.create_docks()
        self.add_widgets()
        self.add_clickevents()
        
        
        self.win.show()
    
    def init_FSM(self):
        self.fsm = FiniteStateMachine()
    
    def create_docks(self):
        self.d1 = Dock("Control Logic", size=(1, 1))     ## give this dock the minimum possible size
        self.d2 = Dock("Dock2 - Console", size=(500,300), closable=True)
        self.d3 = Dock("Dock3", size=(500,400))
        self.d4 = Dock("Dock4 (tabbed) - Plot", size=(500,200))
        self.d5 = Dock("Dock5 - Image", size=(500,200))
        self.d6 = Dock("Dock6 (tabbed) - Plot", size=(500,200))
        self.dock7 = Dock("RealTime - Plot", size=(500,200))
        self.dock8 = Dock("Map",size=(500,300))
        self.dock9 = Dock("MatplotLib", size=(500,300))
        
        self.area.addDock(self.d1, 'left')      
        self.area.addDock(self.d2, 'right')     
        self.area.addDock(self.d3, 'bottom', self.d1)
        self.area.addDock(self.d4, 'right')     
        self.area.addDock(self.d5, 'left', self.d1)  
        self.area.addDock(self.d6, 'top', self.d4)   
        self.area.addDock(self.dock7, 'above', self.d3)
        self.area.addDock(self.dock8, 'right', self.d4)
        self.area.addDock(self.dock9, 'above', self.d2)
        
        ## Test ability to move docks programatically after they have been placed
        self.area.moveDock(self.d4, 'top', self.d2)     ## move d4 to top edge of d2
        self.area.moveDock(self.d6, 'above', self.d4)   ## move d6 to stack on top of d4
        self.area.moveDock(self.d5, 'top', self.d2)     ## move d5 to top edge of d2
    
    def add_widgets(self):
        
        self.label = QtGui.QLabel("""System control functions & Load, Save, Restore functions 
        """)
        self.loadBtn = QtGui.QPushButton('Load Data')
        self.saveBtn = QtGui.QPushButton('Save GUI state')
        self.restoreBtn = QtGui.QPushButton('Restore GUI state')
        self.abortBtn = QtGui.QPushButton('Pause')
        self.abortBtn.setStyleSheet(self.ButtonStyle_red)
        self.logToggleBtn = QtGui.QPushButton('Log Toggle')
        self.logToggleBtn.setStyleSheet(self.ButtonStyle_white)
        self.startBtn = QtGui.QPushButton('Start')
        self.startBtn.setStyleSheet(self.ButtonStyle_green)
        self.originBtn = QtGui.QPushButton('Set Origin')
        self.originBtn.setStyleSheet(self.ButtonStyle_white)

        self.restoreBtn.setEnabled(False)
        
        self.w1 = pg.LayoutWidget()
        self.w1.addWidget(self.label, row=0, col=0,colspan=4)
        self.w1.addWidget(self.loadBtn, row=1,col=0,colspan=4)
        self.w1.addWidget(self.saveBtn, row=2, col=0, colspan=2)
        self.w1.addWidget(self.restoreBtn, row=2, col=2, colspan=2)
        self.w1.addWidget(self.startBtn, row=3, col=0)
        self.w1.addWidget(self.logToggleBtn, row=3, col=1)
        self.w1.addWidget(self.originBtn, row=3, col=2)
        self.w1.addWidget(self.abortBtn, row=3, col=3)
        self.d1.addWidget(self.w1)
        
        self.w2 = pg.console.ConsoleWidget()
        self.d2.addWidget(self.w2)

        ## Hide title bar on dock 3
        # self.d3.hideTitleBar()
        self.w3 = pg.PlotWidget(title="Plot inside dock with no title bar")
        self.w3.plot(np.random.normal(size=100))
        self.d3.addWidget(self.w3)

        self.w4 = pg.PlotWidget(title="Dock 4 plot")
        self.w4.plot(np.random.normal(size=100))
        self.d4.addWidget(self.w4)

        self.w5 = pg.ImageView()
        self.w5.setImage(np.random.normal(size=(5,5)))
        self.d5.addWidget(self.w5)

        self.w6 = pg.PlotWidget(title="Dock 6 plot")
        self.w6.plot(np.random.normal(size=100))
        self.d6.addWidget(self.w6)
        
        # realtime
        self.widget7 = pg.PlotWidget(name='Plot1', title = "RealTime Data")
        self.widget7.setLabel('left', 'Value', units='V')
        self.widget7.setLabel('bottom', 'Time', units='s')
        self.widget7.setXRange(0, 2)
        self.widget7.setYRange(0, 1e-10)
        self.curve7 = self.widget7.plot()
        self.curve7.setPen((200,200,100))
        self.dock7.addWidget(self.widget7)
        
        # Map
        self.widget8 = pg.LayoutWidget()
        
        self.worldmap = folium.Map(
            title="Ottawa",
            zoom_start=13,
            location = self.coord
        )
        folium.TileLayer('Stamen Terrain').add_to(self.worldmap)
        folium.TileLayer('Stamen Toner').add_to(self.worldmap)
        folium.TileLayer('Stamen Water Color').add_to(self.worldmap)
        folium.TileLayer('cartodbpositron').add_to(self.worldmap)
        folium.TileLayer('cartodbdark_matter').add_to(self.worldmap)
        folium.LayerControl().add_to(self.worldmap)
        folium.Marker(location=self.coord,popup='Marker1',tooltip='First Waypoint').add_to(self.worldmap)
        # Save map data to data object
        self.map_data = io.BytesIO()
        self.worldmap.save(self.map_data, close_file=False)
        self.webview = QWebEngineView()
        self.webview.setHtml(self.map_data.getvalue().decode())
        # print(self.map_data.getvalue().decode())
        self.widget8.addWidget(self.webview)
        self.dock8.addWidget(self.widget8)
        
        # Matplot lib plot
        self.widget9 = pg.LayoutWidget()
        self.canvas = MatplotLibCanvas()
        toolbar = NavigationToolbar(self.canvas, self)
        self.widget9.addWidget(toolbar)
        self.widget9.addWidget(self.canvas)
        self.dock9.addWidget(self.widget9)
        
    def rand(self, n):
        data = np.random.random(n)
        data[int(n*0.1):int(n*0.13)] += .5
        data[int(n*0.18)] += 2
        data[int(n*0.1):int(n*0.13)] *= 5
        data[int(n*0.18)] *= 20
        data *= 1e-12
        return data, np.arange(n, n+len(data)) / float(n)
        
    def plot(self):
        
        x,y,z = self.get_xyz_list()
        self.canvas.plot3D(x,y,z)
                
        
    def load(self):
        global filename
        filename, _ = QFileDialog.getOpenFileName(self,"Load Data", "", "All files (*)") 
        print(filename) 
        
        self.plot()
        # Real time graphing https://www.learnpyqt.com/tutorials/plotting-matplotlib/
    
    def get_xyz_list(self):
        global filename
        
        with open(filename) as f:
            array = []
            
            for line in f:
                array.append([float(x) for x in line.split(',')])
            Xarray, Yarray, Zarray = [], [], []
            for i in range(len(array)):
                Xarray.append(array[i][0]) 
                Yarray.append(array[i][1]) 
                Zarray.append(array[i][2]) 
            
            return Xarray, Yarray, Zarray
                
    
    def initialize(self):
        try:
            self.fsm.initialize()
            print('Initialized')
            # Add init logic here
        except Exception as Error:
            print(Error)
            
    def abort(self):
        try:
            self.fsm.abort()
            print('Aborted')
        except Exception as Error:
            print(Error)
            
    def save(self):
        # global state
        self.state = self.area.saveState()
        self.restoreBtn.setEnabled(True)
    
    def restore(self):
        # global state
        self.area.restoreState(self.state)
        
    def add_clickevents(self):
        self.saveBtn.clicked.connect(self.save)   
        self.restoreBtn.clicked.connect(self.restore)
        self.startBtn.clicked.connect(self.initialize)
        self.abortBtn.clicked.connect(self.abort)
        self.loadBtn.clicked.connect(self.load)
    
    def update(self):
        # print(self.__version__)
        
        # Check state
        # print(self.fsm.current_state)
        
        if self.fsm.is_idle:
            pass
        else: 
            yd, xd = self.rand(10000)
            self.curve7.setData(y = yd, x =xd)
            
        
        
    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
            QtGui.QApplication.instance().exec_()
            
    def animation(self):
        # Add stuff here
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(20)
        self.start()



if __name__ == '__main__':
    Window = MainWindow()
    Window.animation()
    

