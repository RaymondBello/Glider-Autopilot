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
import time
import json
import math
import serial
import socket
import numpy as np
from datetime import datetime
import folium
from os.path import dirname, realpath, join

from SERIAL import Comms
from TCP import WS_Manager
from UDP import UDP_Manager
from AVA import AVA

from statemachine import StateMachine, State

import pyqtgraph as pg
import pyqtgraph.console
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *

from PyQt5.QtWidgets import QPushButton,QWidget, QBoxLayout, QVBoxLayout, QFileDialog, QAction
from PyQt5.QtWebEngineWidgets import QWebEngineView

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar




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
    idle = State('Idle', initial=True)
    active = State('Active')
    
    initialize = idle.to(active)
    abort = active.to(idle)


class MainWindow(QWidget):
     
    MainWindowTitle = "UAV Ground Control System"
    __version__ = 0.1
    
    WINDOW_WIDTH = 1500
    WINDOW_HEIGHT = 800
    
    background_color = (36, 37, 41)
    aspect_ratio_locked = True
    
    build_mode = None
    debug_mode = False
    tcp_mode = False
    serial_mode = False
    
    # Button styles
    ButtonStyle_white = "background-color:rgb(255, 255, 255);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_red = "background-color:rgb(200, 0, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_green = "background-color:rgb(0, 155, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_yellow = "background-color:rgb(255, 255, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    
    counter = 0
    
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
        
        # initialize finite state machine
        self.init_FSM()
        
        # Graph variables
        self.random_plot_step = 100
        self.raw_data = []
        self.serial_data = [1] * 19
        self.count = 0

        self.init_network()
        
        self.create_fonts()
        self.create_docks()
        self.add_widgets()
        self.add_clickevents()
        
        self.win.show()
    
    @classmethod
    def from_argv(cls):
        """Class constructor using arguments
        
        Build-Mode Arguments:
            0 : Run in debug mode. Uses random data in all graphs
            1 : Run in TCP Websocket mode
            2 : Run in Serial mode

        Returns:
            cls(): Returns a class initialized using sys.argv arguments
        """
        # Any valid argument is used as build mode specifier, If no valid argument, build mode defaults to debug
        argv = (int(sys.argv[-1]) if int(sys.argv[-1]) < 3 else 0) if sys.argv[-1].isnumeric() else 0
        
        cls.build_mode = argv
        
        cls.debug_mode = True if cls.build_mode == 0 else False
        cls.tcp_mode = True if cls.build_mode == 1 else False
        cls.serial_mode = True if cls.build_mode == 2 else False
        
        return cls()
    
    def init_FSM(self):
        self.fsm = FiniteStateMachine()
    
    def create_fonts(self):
        self.font1 = QtGui.QFont()
        self.font1.setPixelSize(50)
        self.font1.setWeight(100)
        self.font1.setFamily('Fira Code')
        self.font2 = QtGui.QFont()
        self.font2.setPixelSize(35)
        self.font2.setWeight(100)
        self.font2.setFamily('Fira Code')
        self.font3 = QtGui.QFont()
        self.font3.setPixelSize(30)
        self.font3.setWeight(100)
        self.font3.setFamily('Fira Code')
        self.font_gps = QtGui.QFont()
        self.font_gps.setPixelSize(15)
        self.font_gps.setWeight(100)
        self.font_gps.setFamily('Fira Code')
        self.font_gps.setCapitalization(3)
        
    def init_network(self):
        '''
        start up networking interfaces
        '''
        if self.tcp_mode:
            try:
                print("[SET-UP] : Setting up TCP connection")
                self.ws_socket = WS_Manager()
                print("[SET-UP] : TCP Connection Establied!")
                self.AVA_model = AVA(
                    self.ws_socket.targetIP, self.current_state, self.system_state_pool)
                print("[SET-UP] : AVA model setup completed")
            except Exception as identifier:
                print(f"[ERROR] : (setting up ws) {identifier}")
        
        # self.udp_socket = UDP_Manager()
        
    def create_docks(self):
        self.dock1 = Dock("1) Control Logic", size=(1, 1))     ## give this dock the minimum possible size
        self.dock2 = Dock("2) Py Console", size=(500,200), closable=True)
        self.dock3 = Dock("3) Accelerometer", size=(500,400))
        self.dock4 = Dock("4) Gyroscope", size=(500,400))
        self.dock5 = Dock("5) Dock5 - Image", size=(500,400))
        self.dock6 = Dock("6) Dock6 (tabbed) - Plot", size=(500,200))
        self.dock7 = Dock("7) RealTime - Plot", size=(500,200))
        self.dock8 = Dock("8) GCS Map",size=(500,300))
        self.dock9 = Dock("9) MatplotLib", size=(500,300))
        
        self.area.addDock(self.dock1, 'left')      
        self.area.addDock(self.dock2, 'right', self.dock1)     
        self.area.addDock(self.dock3, 'top', self.dock2)
        self.area.addDock(self.dock4, 'top',self.dock2)     
        self.area.addDock(self.dock5, 'top', self.dock2)  
        # self.area.addDock(self.dock6, 'right', self.dock5)   
        # self.area.addDock(self.dock7, 'bottom', self.dock3)
        # self.area.addDock(self.dock8, 'right', self.dock4)
        self.area.addDock(self.dock8, 'bottom', self.dock1)
        
        
        # Test ability to move docks programatically after they have been placed
        # self.area.moveDock(self.dock4, 'top', self.dock2)     ## move dock4 to top edge of dock2
        # self.area.moveDock(self.dock6, 'above', self.dock4)   ## move dock6 to stack on top of dock4
        # self.area.moveDock(self.dock5, 'top', self.dock2)     ## move dock5 to top edge of dock2
        # self.area.moveDock(self.dock1, 'above', self.dock3)
    
    def add_widgets(self):
        # Label
        self.label = QtGui.QLabel("""System control functions & Load, Save, Restore functions 
        """)
        # Buttons
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
        
        # widget 1 (Main widget)
        self.widget1 = pg.LayoutWidget()
        self.widget1.addWidget(self.label, row=0, col=0,colspan=4)
        self.widget1.addWidget(self.loadBtn, row=1,col=0,colspan=4)
        self.widget1.addWidget(self.saveBtn, row=2, col=0, colspan=2)
        self.widget1.addWidget(self.restoreBtn, row=2, col=2, colspan=2)
        self.widget1.addWidget(self.startBtn, row=3, col=0)
        self.widget1.addWidget(self.logToggleBtn, row=3, col=1)
        self.widget1.addWidget(self.originBtn, row=3, col=2)
        self.widget1.addWidget(self.abortBtn, row=3, col=3)
        self.dock1.addWidget(self.widget1)
        
        
        # widget 2 (console)
        self.widget2 = pg.console.ConsoleWidget()
        self.dock2.addWidget(self.widget2)


        ## widget 3 (Accelerometer Graph)
        self.widget3 = pg.PlotWidget(title="imu_accel")
        self.widget3.setDownsampling(mode="peak")
        self.widget3.setClipToView(True)
        self.widget3.addLegend(offset=(1, 1))
        self.widget3.setRange(xRange=[-200, 0])
        self.widget3.setLimits(xMax=0)
        self.widget3_curve0 = self.widget3.plot(
            pen=(150, 0, 0),
            name="Ax",
            labels={
                "left": self.widget3.setLabel("left", text="Acceleration ", units="m/s^2"),
                "bottom": self.widget3.setLabel("bottom", text="Time", units="s"),
            },)
        self.widget3_curve1 = self.widget3.plot(
            pen=(0, 150, 0),
            name="Ay",
        )
        self.widget3_curve2 = self.widget3.plot(
            pen=(0, 0, 150),
            name="Az",
        )
        self.dock3.addWidget(self.widget3)
        # self.dock3.hideTitleBar()
        # buffer for dock 3 (accel)
        self.data3_0 = np.empty(200)
        self.data3_1 = np.empty(200)
        self.data3_2 = np.empty(200)
        self.ptr1 = 0
        
        
        # widget 4 (Gyroscope Graph)
        self.widget4 = pg.PlotWidget(title="imu_gyro")
        self.widget4.setDownsampling(mode="peak")
        self.widget4.setClipToView(True)
        self.widget4.addLegend(offset=(1, 1))
        self.widget4.setRange(xRange=[-200, 0])
        self.widget4.setLimits(xMax=0)
        self.widget4_curve0 = self.widget4.plot(
            pen=(150, 0, 0),
            name="Gx",
            labels={
                "left": self.widget4.setLabel("left", text="Angular Accel ", units="rad/s2"),
                "bottom": self.widget4.setLabel("bottom", text="Time", units="s"),
            },
        )
        self.widget4_curve1 = self.widget4.plot(
            pen=(0, 150, 0),
            name="Gy"
        )
        self.widget4_curve2 = self.widget4.plot(
            pen=(0, 0, 150),
            name="Gz"
        )
        self.dock4.addWidget(self.widget4)
        
        self.data4_0 = np.empty(200)
        self.data4_1 = np.empty(200)
        self.data4_2 = np.empty(200)
        self.ptr2 = 0
        



        self.widget5 = pg.ImageView()
        self.widget5.setImage(np.random.normal(size=(5,5)))
        self.dock5.addWidget(self.widget5)

        self.widget6 = pg.PlotWidget(title="Dock 6 plot")
        self.widget6.plot(np.random.normal(size=100))
        self.dock6.addWidget(self.widget6)
        
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
            zoom_start=15,
            # tiles='cartodbpositron',
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
    
    def generate_data(self, data):
        """Generates sin waves to be displayed for UI debugging and developmentpython -m pyqtgraph.examples
        available only in DEBUG MODE (self.build_mode = 0)

        Args:
            data (list): Array of data shared by all the graphs

        Returns:
            list: Array of sin waves with varying phase and polarity
        """
        self.counter = self.counter if self.counter < 360 else 0
        self.counter += 1
        
        return [2*math.sin(self.counter + (idx*180 if bool(idx%2) else -idx*90) ) for idx, val in enumerate(self.serial_data)]
    
    def tcp_handleshack(self, commandType: str, commandData: str):
        ''' Takes the command type and command data and returns raw data'''
        self.ws_socket.send_data(commandType, commandData)
        self.raw_data = self.ws_socket.receive_data()

        return self.raw_data
    
    def update_widget3(self,accel_values):
        if self.ptr1 < self.random_plot_step:
            self.data3_0[self.ptr1] = np.random.normal()
            self.data3_1[self.ptr1] = np.random.normal()
            self.data3_2[self.ptr1] = np.random.normal()
            
        else:
            if len(accel_values) == 3:
                try:
                    data3_0_value = round(float(accel_values[0]),3)
                    data3_1_value = round(float(accel_values[1]),3)
                    data3_2_value = round(float(accel_values[2]),3)
                    
                    self.data3_0[self.ptr1] = data3_0_value
                    self.data3_1[self.ptr1] = data3_1_value
                    self.data3_2[self.ptr1] = data3_2_value
                except ValueError as error:
                    print(f"[VALUE ERROR]: {error}")
                    
        self.ptr1 += 1
        
        if self.ptr1 >= self.data3_0.shape[0]:
            tmp1_0 = self.data3_0
            tmp1_1 = self.data3_1
            tmp1_2 = self.data3_2
            
            self.data3_0 = np.empty(self.data3_0.shape[0] * 2)
            self.data3_1 = np.empty(self.data3_1.shape[0] * 2)
            self.data3_2 = np.empty(self.data3_2.shape[0] * 2)
            
            self.data3_0[: tmp1_0.shape[0]] = tmp1_0
            self.data3_1[: tmp1_1.shape[0]] = tmp1_1
            self.data3_2[: tmp1_2.shape[0]] = tmp1_2
        
        self.widget3_curve0.setData(self.data3_0[:self.ptr1])
        self.widget3_curve1.setData(self.data3_1[:self.ptr1])
        self.widget3_curve2.setData(self.data3_2[:self.ptr1])
        
        self.widget3_curve0.setPos(-self.ptr1, 0)
        self.widget3_curve1.setPos(-self.ptr1, 0)
        self.widget3_curve2.setPos(-self.ptr1, 0)
    
    def update_widget4(self, gyro_values):
        if self.ptr2 < self.random_plot_step:
            self.data4_0[self.ptr2] = np.random.normal()
            self.data4_1[self.ptr2] = np.random.normal()
            self.data4_2[self.ptr2] = np.random.normal() 
            
        else:
            if len(gyro_values) == 3:
                try:
                    data4_0_value = round(float(gyro_values[0]), 3)
                    data4_1_value = round(float(gyro_values[1]), 3)
                    data4_2_value = round(float(gyro_values[2]), 3)
                    
                    self.data4_0[self.ptr2] = data4_0_value
                    self.data4_1[self.ptr2] = data4_1_value
                    self.data4_2[self.ptr2] = data4_2_value
                except ValueError as error:
                    print(f"[VALUE ERROR]: {error}")
                    
        self.ptr2 += 1
        
        if self.ptr2 >= self.data4_0.shape[0]:
            tmp1_0 = self.data4_0
            tmp1_1 = self.data4_1
            tmp1_2 = self.data4_2
            
            self.data4_0 = np.empty(self.data4_0.shape[0]*2)
            self.data4_1 = np.empty(self.data4_1.shape[0]*2)
            self.data4_2 = np.empty(self.data4_2.shape[0]*2)
            
            self.data4_0[: tmp1_0.shape[0]] = tmp1_0
            self.data4_1[: tmp1_1.shape[0]] = tmp1_1
            self.data4_2[: tmp1_2.shape[0]] = tmp1_2
        
        self.widget4_curve0.setData(self.data4_0[:self.ptr2])
        self.widget4_curve1.setData(self.data4_1[:self.ptr2])
        self.widget4_curve2.setData(self.data4_2[:self.ptr2])
        
        self.widget4_curve0.setPos(-self.ptr2, 0)
        self.widget4_curve1.setPos(-self.ptr2, 0)
        self.widget4_curve2.setPos(-self.ptr2, 0)
        
                    
    def update(self):
        # print(self.__version__)
        
        # Check state
        # print(self.fsm.current_state)
        
        # Depending on current build mode
        if self.tcp_mode:
            self.serial_data = self.tcp_handleshack("STATE", self.current_state)
            self.serial_data = self.AVA_model.update(self.serial_data, self.current_state)
            
        if self.debug_mode:
            self.serial_data = self.generate_data(self.serial_data)
        
        if self.serial_mode:
            print("Serial Mode")
        
        # Display based on state
        if not self.fsm.is_idle:
            yd, xd = self.rand(10000)
            self.curve7.setData(y = yd, x =xd)
            
            self.update_widget3(self.serial_data[0:3])
            self.update_widget4(self.serial_data[3:6])
        else: 
            pass
        
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
    Window = MainWindow.from_argv()
    Window.animation()
    

