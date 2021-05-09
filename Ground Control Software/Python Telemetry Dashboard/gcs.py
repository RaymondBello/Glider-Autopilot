#!/usr/bin/python3
# -*- coding: utf-8 -*-

__author__ = "Ray Bello"
__copyright__ = "Copyright 2020, RayTech"
__version__ = "0.0.1"
__maintainer__ = "Ray"
__status__ = "Development"


import io
import sys
import time
import json
import math
import serial
import socket
import json
import numpy as np
from datetime import datetime
# from pyqtlet import L, MapWidget
from os.path import dirname, realpath, join

from submodules.model import *
from submodules.generate_traj import *
from submodules.module_serial import Comms
from submodules.module_tcp import WS_Manager
from submodules.module_udp import UDP_Manager

from statemachine import StateMachine, State

import pyqtgraph as pg
import pyqtgraph.console
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *

from PyQt5.QtWidgets import QPushButton,QWidget, QBoxLayout, QVBoxLayout, QFileDialog, QAction, QComboBox, QLineEdit, QTextEdit, QGroupBox, QGridLayout, QLabel
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtGui import QStandardItemModel, QStandardItem

from PyQt5.QtOpenGL import QGL, QGLFormat, QGLWidget
from PyQt5.QtSvg import QGraphicsSvgItem

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar


class MatplotLibCanvas(FigureCanvas):
    
    def __init__(self, width=10,height=10, dpi=100):
        self.fig = Figure(figsize=(width,height),dpi=dpi)
        FigureCanvas.__init__(self,self.fig)
    
    def plot2D(self, xarray, yarray):
        self.fig.clear()
        self.axe2D = self.fig.add_subplot(111)
        self.axe2D.plot(xarray,yarray,'ok')
        self.axe2D.plot(xarray,yarray,'r-')
        self.axe2D.set_xlabel('X')
        self.axe2D.set_ylabel('Y')
        self.axe2D.set_zlabel('Z')
        self.draw()
    
    def plot3D(self, xarray,yarray,zarray):
        self.fig.clear()
        self.axe3D = self.fig.add_subplot(111, projection='3d')
        self.axe3D.plot_trisurf(xarray,yarray,zarray, color='red', alpha=0.6,edgecolor='red',shade=1)
        self.axe3D.plot(xarray,yarray,zarray,'ok')
        self.axe3D.set_xlabel('X')
        self.axe3D.set_ylabel('Y')
        self.axe3D.set_zlabel('Z')
        self.draw()


class FiniteStateMachine(StateMachine):
    ''
    idle = State('Idle', initial=True)
    active = State('Active')
    
    initialize = idle.to(active)
    abort = active.to(idle)


class MainWindow(QWidget):
    '''
    Main Window from Ground Control System
    '''
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
    sim_mode = False
    
    # Button styles
    ButtonStyle_white = "background-color:rgb(255, 255, 255);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_red = "background-color:rgb(200, 0, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_green = "background-color:rgb(0, 155, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    ButtonStyle_yellow = "background-color:rgb(255, 255, 0);color:rgb(0,0,0);font-size:18px;font-weight:light"
    
    counter = 0
    
    # Initial Map coordinates
    coord = (45.40415841728934, -75.70669454423376)
    
    combobox_data = {
        'Debug':['Autogenerated'],
        'Serial':[],
        'Network':['WebSocket', 'UDP'],
    }
    port_list = serial.tools.list_ports.comports()
    for port in sorted(port_list):
            # List of ports: https://stackoverflow.com/a/52809180
            combobox_data['Serial'].append("{}".format(port).split('-')[0].strip())
    
    
    def __init__(self):
        self.app = pg.mkQApp()
        self.win = QtGui.QMainWindow()
        self.area = DockArea()
        
        self.win.setCentralWidget(self.area)
        self.win.resize(self.WINDOW_WIDTH, self.WINDOW_HEIGHT)
        
        self.win.setWindowTitle(self.MainWindowTitle)
        super().__init__()
        
        # initialize finite state machine
        self.init_FSM()
        self.debug_mode = True  
        
        # Graph variables
        self.random_plot_step = 100
        self.raw_data = []
        self.serial_data = [1] * 19
        self.count = 0

        self.selected_portname = None
        self.selected_mode = None
        
        self.init_network()
        self.init_serial_handler()
        
        self.create_fonts()
        self.create_docks()
        self.add_widgets()
        self.add_clickevents()
        
        
        self.win.show()
        # self.win.showMaximized()
        
        # framerate variables
        self.prev_sec = 0
        self.fps_count = 0
        
        # Initialize communication with Java Graph Viewer
        self.init_javaviewer_connection()
        self.start_javaviewer_app()
        
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
        self.font2.setPixelSize(25)
        self.font2.setWeight(100)
        self.font2.setFamily('Fira Code')
        self.font3 = QtGui.QFont()
        self.font3.setPixelSize(20)
        self.font3.setWeight(100)
        self.font3.setFamily('Fira Code')
        self.font_gps = QtGui.QFont()
        self.font_gps.setPixelSize(15)
        self.font_gps.setWeight(100)
        self.font_gps.setFamily('Fira Code')
        self.font_gps.setCapitalization(3)
        self.font_combobox = QtGui.QFont()
        self.font_combobox.setPixelSize(12)
        self.font_combobox.setFamily('Fira Code')
        self.font_textInput = QtGui.QFont()
        self.font_textInput.setPixelSize(17)
        self.font_textInput.setFamily('Fira Code')
        
    def init_network(self):
        '''
        start up networking interfaces
        '''
        if self.tcp_mode:
            try:
                print("[SET-UP] : Setting up TCP connection")
                self.ws_socket = WS_Manager()
                print("[SET-UP] : TCP Connection Establied!")
            except Exception as identifier:
                print(f"[ERROR] : (setting up ws) {identifier}")
        
        # self.udp_socket = UDP_Manager()
        
    def init_serial_handler(self):
        self.serial_handler = Comms(baudrate=115200,timeout=2)
        
    def create_docks(self):
        self.dock1 = Dock("1) Control Logic", size=(1, 1))     ## give this dock the minimum possible size
        self.dock2 = Dock("2) Py Console", size=(500,500), closable=True)
        self.dock6 = Dock("6) Aircraft Data", size=(200,400))
        self.dock7 = Dock("7) Serial Monitor", size=(500,400))
        self.dock8 = Dock("8) GCS Variables",size=(500,300))
        self.dock9 = Dock("9) MatplotLib", size=(500,300))
        self.dock10 = Dock("10) 3D Trajectory",size=(300,400))
        self.dock11 = Dock("11) Primary Flight Display", size=(900,1100))
        self.dock12 = Dock("12) Instrument Panel", size=(300,400))
        
        self.area.addDock(self.dock1, 'left')      
        self.area.addDock(self.dock2, 'right', self.dock1)
        self.area.addDock(self.dock6, 'bottom', self.dock1)     
        self.area.addDock(self.dock8, 'bottom', self.dock2)
        self.area.addDock(self.dock12, 'above', self.dock8)
        self.area.addDock(self.dock7, 'above', self.dock12)
        self.area.addDock(self.dock9, 'above', self.dock2)
        self.area.addDock(self.dock10, 'above', self.dock9)
        self.area.addDock(self.dock11, 'above', self.dock10)
        
        # self.area.moveDock(self.dock1, 'above', self.dock12)
        # self.area.moveDock(self.dock10, 'above', self.dock8)     # move dock4 to top edge of dock2
        
    def update_mode_comboBox(self, index):
        indx = self.model.index(index, 0, self.comboModes.rootModelIndex())
        self.comboPorts.setRootModelIndex(indx)
        self.comboPorts.setCurrentIndex(0)
        
    def get_selected_portname(self):
        self.selected_mode = self.comboModes.currentText()
        self.selected_portname = self.comboPorts.currentText()
        self.serialText.append(f"Mode:{self.selected_mode}, Port:{self.selected_portname}")
        
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
        self.mapBtn = QtGui.QPushButton('Load Map')

        self.restoreBtn.setEnabled(False) # Greys out Restore Button on boot
        
        # Combo Box Set-up
        self.model = QStandardItemModel()
        self.mode_label = QtGui.QLabel("""Select Mode""")
        self.mode_label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight)
        self.port_label = QtGui.QLabel("""Select Data Source""")
        self.port_label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight)
        self.comboModes = QComboBox()
        self.comboModes.setModel(self.model)  
        self.comboPorts = QComboBox()
        self.comboPorts.setModel(self.model) 
        # Add dropdown data
        for k,v in self.combobox_data.items():
            mode = QStandardItem(k)
            self.model.appendRow(mode)
            for value in v:
                port = QStandardItem(value)
                mode.appendRow(port)
        
        self.update_mode_comboBox(0)
        self.selected_mode = self.comboModes.currentText()
        self.selected_portname = self.comboPorts.currentText()
        
        
        # widget 1 (Main widget)
        self.widget1 = pg.LayoutWidget()
        self.widget1.addWidget(self.label, row=0, col=0,colspan=4)
        self.widget1.addWidget(self.loadBtn, row=1,col=0,colspan=3)
        self.widget1.addWidget(self.mapBtn, row=1, col=3)
        self.widget1.addWidget(self.saveBtn, row=2, col=0, colspan=2)
        self.widget1.addWidget(self.restoreBtn, row=2, col=2, colspan=2)
        self.widget1.addWidget(self.startBtn, row=3, col=0)
        self.widget1.addWidget(self.logToggleBtn, row=3, col=1)
        self.widget1.addWidget(self.originBtn, row=3, col=2)
        self.widget1.addWidget(self.abortBtn, row=3, col=3)
        self.widget1.addWidget(self.mode_label, row=4, col=0)
        self.widget1.addWidget(self.comboModes, row=4, col=1)
        self.widget1.addWidget(self.port_label, row=4,col=2)
        self.widget1.addWidget(self.comboPorts, row=4, col=3)
        
        self.textInput = QLineEdit()
        self.textInput.setFixedHeight(30)
        self.textInput.setFont(self.font_textInput)
        
        self.sendCmdBtn = QtGui.QPushButton('Send Cmd')
        self.sendCmdBtn.setStyleSheet(self.ButtonStyle_white)
        
        self.widget1.addWidget(self.textInput,row=5,col=0,colspan=3)
        self.widget1.addWidget(self.sendCmdBtn, row=5,col=3)
        
        self.dock1.addWidget(self.widget1)
        
        
        # widget 2 (console)
        self.widget2 = pg.console.ConsoleWidget()
        self.dock2.addWidget(self.widget2)
        self.dock2.hideTitleBar()


        # widget 6 (Aircraft Data)
        self.widget6 = pg.LayoutWidget()

        self.stateGraphic = pg.PlotWidget(title='state')
        self.stateGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.stateGraphic.hideAxis('bottom')
        self.stateGraphic.hideAxis('left')
        self.texttoState = pg.TextItem(f"idle", anchor=(0.5, 0.5), color='w')
        self.texttoState.setFont(self.font2)
        self.stateGraphic.addItem(self.texttoState)
        
        self.timeGraphic = pg.PlotWidget(title='current_time')
        self.timeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.timeGraphic.hideAxis('bottom')
        self.timeGraphic.hideAxis('left')
        now = datetime.now()
        self.texttoTime = pg.TextItem(f"{now.strftime('%H:%M:%S')}", anchor=(0.5, 0.5), color='w')
        self.texttoTime.setFont(self.font2)
        self.timeGraphic.addItem(self.texttoTime)
        
        self.batteryGraphic = pg.PlotWidget(title='battery voltage')
        self.batteryGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.batteryGraphic.hideAxis('bottom')
        self.batteryGraphic.hideAxis('left')
        self.texttoBattery = pg.TextItem(f"{11.2} V", anchor=(0.5, 0.5), color='w')
        self.texttoBattery.setFont(self.font2)
        self.batteryGraphic.addItem(self.texttoBattery)
        
        self.pitchGraphic = pg.PlotWidget(title='pitch_imu')
        self.pitchGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.pitchGraphic.hideAxis('bottom')
        self.pitchGraphic.hideAxis('left')
        self.texttoPitch = pg.TextItem(f"0.0°", anchor=(0.5, 0.5), color='w')
        self.texttoPitch.setFont(self.font2)
        self.pitchGraphic.addItem(self.texttoPitch)
        
        self.rollGraphic = pg.PlotWidget(title='roll_imu')
        self.rollGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.rollGraphic.hideAxis('bottom')
        self.rollGraphic.hideAxis('left')
        self.texttoRoll = pg.TextItem(f"0.0°", anchor=(0.5, 0.5), color='w')
        self.texttoRoll.setFont(self.font2)
        self.rollGraphic.addItem(self.texttoRoll)
        
        self.yawGraphic = pg.PlotWidget(title='roll_imu')
        self.yawGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.yawGraphic.hideAxis('bottom')
        self.yawGraphic.hideAxis('left')
        self.texttoYaw = pg.TextItem(f"0.0°", anchor=(0.5, 0.5), color='w')
        self.texttoYaw.setFont(self.font2)
        self.yawGraphic.addItem(self.texttoYaw)
        
        self.altitudeGraphic = pg.PlotWidget(title="altitude")
        self.altitudeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.altitudeGraphic.hideAxis('bottom')
        self.altitudeGraphic.hideAxis('left')
        self.texttoAltitude = pg.TextItem(f"0.01m", anchor=(0.5, 0.5), color='w')
        self.texttoAltitude.setFont(self.font2)
        self.altitudeGraphic.addItem(self.texttoAltitude)
        
        self.GPSDataGraphic = pg.PlotWidget(title="gps_data")
        self.GPSDataGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.GPSDataGraphic.hideAxis('bottom')
        self.GPSDataGraphic.hideAxis('left')
        self.texttoGPS = pg.TextItem("null", anchor=(0.5, 0.5), color='w')
        self.texttoGPS.setFont(self.font3)
        self.GPSDataGraphic.addItem(self.texttoGPS)
        
        self.widget6.addWidget(self.stateGraphic,row=0, col=0)
        self.widget6.addWidget(self.timeGraphic,row=0, col=1)
        self.widget6.addWidget(self.batteryGraphic,row=0, col=2)
        self.widget6.addWidget(self.pitchGraphic,row=1, col=0)
        self.widget6.addWidget(self.rollGraphic,row=1, col=1)
        self.widget6.addWidget(self.yawGraphic,row=1, col=2)
        self.widget6.addWidget(self.altitudeGraphic,row=2, col=0)
        self.widget6.addWidget(self.GPSDataGraphic,row=2, col=1,colspan=2)
        self.dock6.addWidget(self.widget6)
        
        
        # realtime
        self.widget7 = pg.LayoutWidget()
        
        self.serialText = QTextEdit()
        self.serialText.setReadOnly(True)
        self.serialText.setFont(self.font_textInput)
        self.widget7.addWidget(self.serialText)
        
        
        self.dock7.addWidget(self.widget7)
        
        
        # Load PyLeaflet Map
        # self.load_map()  # Loads map and disables button, uncomment to manually load map
        
        
        # Matplot lib plot
        self.widget9 = pg.LayoutWidget()
        self.canvas = MatplotLibCanvas()
        toolbar = NavigationToolbar(self.canvas, self)
        self.widget9.addWidget(toolbar)
        self.widget9.addWidget(self.canvas)
        self.dock9.addWidget(self.widget9)
        
        
        # 3D Trajectory Plot
        self.widget10 = gl.GLViewWidget()
        self.widget10.setBackgroundColor((30,30,30))             # Black
        # self.widget10.setBackgroundColor((255,255,255))       # White
        # self.widget10.setBackgroundColor((235,235,235))       # Light Grey
        self.widget10.setCameraPosition(distance=17, azimuth=45, elevation=25)
        
        grid = gl.GLGridItem(color=(0,0,0,1))
        grid.scale(5,5,1) 
        self.widget10.addItem(grid)
        
        model = Model("resources/glider.obj")
        meshdata = gl.MeshData(vertexes=model.returnMesh())
        self.glider_mesh = gl.GLMeshItem(meshdata=meshdata, color=(255, 255, 255, 2), smooth=False, shader='shaded', glOptions='opaque')
        
        # Initialize Simulation related Objects
        h_initial = 0
        self.glider_mesh.translate(0,0,h_initial)
        
        # Load trajectory
        gen = gentraj(3)
        traj = gen.traj
     
        # # Visualization of the Trajectory
        pts = np.vstack([traj.X,traj.Y,traj.Z]).transpose()
        plt = gl.GLLinePlotItem(pos=pts,color=pg.glColor(('r')), width=2,antialias=True)
        
        self.widget10.addItem(plt)
        self.widget10.addItem(self.glider_mesh)
        self.dock10.addWidget(self.widget10)
        
        
        # Primary Flight Display (Using HTML)
        self.widget11 = pg.LayoutWidget()
        self.gcs = QWebEngineView()
        
        with open('gcs.html','r') as f:
            raw_html = f.read()
            self.gcs.setHtml(raw_html)
        # self.gcs.setHtml()
        # self.gcs.setUrl(QUrl("www.google.com"))
        self.widget11.addWidget(self.gcs)
        self.dock11.addWidget(self.widget11)
        
        
        # Text Input
        self.widget12 = pg.LayoutWidget()
        # self.widget12 = QGroupBox()
        btn = QPushButton("Pause View")
        btn1 = QPushButton("Start View")
        
        self.label1 = QLabel("Yes")
        
        self.paramater_view = QTextEdit()
        self.paramater_view.setReadOnly(True)
        self.paramater_view.setText("Empty")
        
        self.widget12.addWidget(self.paramater_view,0,0,4,1)
        self.widget12.addWidget(btn,0,1)
        self.widget12.addWidget(btn1,1,1)
        
        self.dock12.addWidget(self.widget12)
        
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
        self.serialText.append(str(filename))
        
        self.plot()
        # Real time graphing https://www.learnpyqt.com/tutorials/plotting-matplotlib/
    
    def load_map(self):
        # Map
        self.widget8 = pg.LayoutWidget()
        self.mapWidget= MapWidget()
        self.widget8.addWidget(self.mapWidget)
        self.map = L.map(self.mapWidget)
        self.map.setView([45.41, -75.7], zoom=13)
        
        L.tileLayer("https://api.mapbox.com/styles/v1/raybell/ckhfmq5j30ad71anskil9i9tz/tiles/256/{z}/{x}/{y}@2x?access_token=pk.eyJ1IjoicmF5YmVsbCIsImEiOiJja2hjazhyd2wwN2l1MnN0M3RtdGd2aTRoIn0.4mpZFa8_T8XfLagyn7EsnQ").addTo(self.map)
        
        self.marker = L.marker([45.41, -75.7])
        self.marker.bindPopup('Maps are a treasure.')
        self.marker.bindTooltip('Test')
        self.map.addLayer(self.marker)
        
        self.dock8.addWidget(self.widget8)
        
        # Button Logic
        self.mapBtn.setEnabled(False)

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
    
    def setModeAndPort(self,selected_mode,selected_port):
        
        if selected_mode == 'Serial':
            self.serial_mode = True
            self.debug_mode = False
            self.tcp_mode = False
            try:
                self.serial_handler.connect( self.selected_portname ) 
                self.serialText.append(f"Is {self.selected_portname} open? {self.serial_handler.isOpen()}")
            except serial.serialutil.SerialException as error:
                self.serialText.append(str(error))
                
        if selected_mode == 'Debug':
            self.serial_mode = False
            self.debug_mode = True
            self.tcp_mode = False
                
    def initialize(self):
        try:
            self.fsm.initialize()
            # Add init logic here
            self.serialText.append(f"Mode: {self.selected_mode}, Port: {self.selected_portname}")
            self.setModeAndPort(self.selected_mode, self.selected_portname)
            
        except Exception as Error:
            self.serialText.append(str(Error))
            
    def abort(self):
        try:
            self.fsm.abort()
        except Exception as Error:
            self.serialText.append(str(Error))
            
    def save(self):
        # global state
        self.state = self.area.saveState()
        self.restoreBtn.setEnabled(True)
    
    def restore(self):
        # global state
        self.area.restoreState(self.state)
    
    # Color Switch Counter
    i = 0
    
    def send_cmd(self):
        textBuffer = self.textInput.text() + ' \n'
        self.serialText.append(f"Command Sent: {str.encode(textBuffer)}")
        self.textInput.clear()
        
        if self.serial_mode:
            self.serial_handler.sendData(textBuffer)
        
        if self.i%3 == 0:
            self.sendCmdBtn.setStyleSheet(self.ButtonStyle_red)
        elif self.i%3 == 1:
            self.sendCmdBtn.setStyleSheet(self.ButtonStyle_yellow)
        elif self.i%3 == 2:
            self.sendCmdBtn.setStyleSheet(self.ButtonStyle_green)
            
        self.i +=1
        if self.i == 3:
            self.i = 0
        
    def add_clickevents(self):
        self.saveBtn.clicked.connect(self.save)   
        self.restoreBtn.clicked.connect(self.restore)
        self.startBtn.clicked.connect(self.initialize)
        self.abortBtn.clicked.connect(self.abort)
        self.loadBtn.clicked.connect(self.load)
        self.mapBtn.clicked.connect(self.load_map)
        self.sendCmdBtn.clicked.connect(self.send_cmd)
        self.comboModes.currentIndexChanged.connect(self.update_mode_comboBox)
        self.comboPorts.currentTextChanged.connect(self.get_selected_portname)
        
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
                           
    def update_widget6(self, aircraft_values):
        
        # clear current values
        self.stateGraphic.removeItem(self.texttoState)
        self.timeGraphic.removeItem(self.texttoTime)
        self.batteryGraphic.removeItem(self.texttoBattery)
        self.pitchGraphic.removeItem(self.texttoPitch)
        self.rollGraphic.removeItem(self.texttoRoll)
        self.yawGraphic.removeItem(self.texttoYaw)
        self.altitudeGraphic.removeItem(self.texttoAltitude)
        
        self.GPSDataGraphic.removeItem(self.texttoGPS)
        
        # Add new text values
        self.texttoState = pg.TextItem(str(self.fsm.current_state.value), anchor=(0.5, 0.5), color='w') # update state
        now = datetime.now()
        self.texttoTime = pg.TextItem(f"{now.strftime('%H:%M:%S')}", anchor=(0.5, 0.5), color="w") # update time
        self.texttoBattery = pg.TextItem(f"{str(round(float(aircraft_values[0]),2))} V", anchor=(0.5, 0.5), color='w') # update battery
        self.texttoPitch = pg.TextItem(f"{str(round(float(aircraft_values[0]),2))}°", anchor=(0.5, 0.5), color='w')
        self.texttoRoll = pg.TextItem(f"{str(round(float(aircraft_values[1]),2))}°", anchor=(0.5, 0.5), color='w')
        self.texttoYaw = pg.TextItem(f"{str(round(float(aircraft_values[2]),2))}°", anchor=(0.5, 0.5), color='w')
        self.texttoAltitude = pg.TextItem(f"{str(round(float(aircraft_values[0]),2))}°", anchor=(0.5, 0.5), color='w')
        
        self.texttoGPS = pg.TextItem(f"{str(round(float(aircraft_values[0]),2))}°", anchor=(0.5, 0.5), color='w')
        
        # set text value font
        self.texttoState.setFont(self.font2)
        self.texttoTime.setFont(self.font2)
        self.texttoBattery.setFont(self.font2)
        self.texttoPitch.setFont(self.font2)
        self.texttoRoll.setFont(self.font2)
        self.texttoYaw.setFont(self.font2)
        self.texttoAltitude.setFont(self.font2)
        
        self.texttoGPS.setFont(self.font2)
        
        # add text back to graphic
        self.stateGraphic.addItem(self.texttoState)
        self.timeGraphic.addItem(self.texttoTime)
        self.batteryGraphic.addItem(self.texttoBattery)
        self.pitchGraphic.addItem(self.texttoPitch)
        self.rollGraphic.addItem(self.texttoRoll)
        self.yawGraphic.addItem(self.texttoYaw)
        self.altitudeGraphic.addItem(self.texttoAltitude)
        
        self.GPSDataGraphic.addItem(self.texttoGPS)
        
    def update_essential(self):
        self.stateGraphic.removeItem(self.texttoState)
        self.timeGraphic.removeItem(self.texttoTime)
        
        self.texttoState = pg.TextItem(str(self.fsm.current_state.value), anchor=(0.5, 0.5), color='w')
        now = datetime.now()
        self.texttoTime = pg.TextItem(f"{now.strftime('%H:%M:%S')}", anchor=(0.5, 0.5), color="w")
        
        self.texttoState.setFont(self.font2)
        self.texttoTime.setFont(self.font2)
        
        self.stateGraphic.addItem(self.texttoState)
        self.timeGraphic.addItem(self.texttoTime)
    
    def update_parameter_viewer(self, packet_data):
        
        
        # parameter_text_struct = f"MODE MONITORING\nFC Mode : {str(packet_data[0])}\nFC Mode : {str(packet_data[0])}\n1"
        
        
        # parameter_text_struct = f""" this is a very/
        # long string if I had the
        # energy to type more and more ..."""
        
        parameter_text_struct = (
            f"MODE MONITORING\t\tFLIGHT VARIABLES\n"
            f"FC Mode: {packet_data[0]:.2f}\t\timu_pitch:   {packet_data[0]:.2f}\n"
            f"ACS Mode: {packet_data[0]:.2f}\t\timu_roll:   {packet_data[0]:.2f}\n"
            f"GCS Mode: {self.selected_mode}\t\timu_yaw:   {packet_data[0]:.2f}\n"
        )
        
        if self.selected_mode == "Debug":
            if len(packet_data) == 19:
                self.paramater_view.setText(parameter_text_struct)
        
        if self.selected_mode == "Serial":
            self.paramater_view.setText(parameter_text_struct)
        
        
        if self.selected_mode == "Network":
            self.paramater_view.setText("Nothing")
        
        
    
    def init_javaviewer_connection(self):
        self.UDP_IP = "192.168.0.30"
        self.UDP_PORT = 8080
        self.MESSAGE = "10,10,10,10\r\n"
        print("UDP target IP:", self.UDP_IP)
        print("UDP target port:", self.UDP_PORT)
        print("message:", self.MESSAGE)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.sock.sendto(bytes(self.MESSAGE, "utf-8"), (self.UDP_IP, self.UDP_PORT))
        pass
    
    def start_javaviewer_app(self):
        pass
    
    def mode_arbitration(self):
        if self.tcp_mode:
            serial_data = self.tcp_handleshack("STATE", self.current_state)
            return serial_data
        
        if self.sim_mode:
            self.serialText.append("running sim mode")
            
        if self.debug_mode:
            serial_data = self.generate_data(self.serial_data)
            return serial_data
        
        if self.serial_mode:
            try:
                serial_data = None
                if self.serial_handler.ser.inWaiting():
                    serial_data = self.serial_handler.getData()
                    # print(serial_data)
                    
                return serial_data
                
            except Exception as error:
                self.serialText.append(f"Serial error: {error}" )
                return [0 for i in range(20)]
    
    def regulate_fps(self, delta, fps=60):
        period_ms = delta * 1000    # sec->ms
        
        # print(f"period {period_ms} :count {self.fps_count}")
        if self.fps_count >=(1/fps)*1000:
            self.fps_count = 0
            return True
        else:
            self.fps_count += period_ms
            return False
        
    def update(self):
        '''
        called to update system variable
        '''
        # regulate application fps
        now = time.perf_counter()
        delta_sec = now-self.prev_sec
        self.prev_sec = now
        
        if self.regulate_fps(delta_sec,fps=30):
            # Check state
            # print(self.fsm.current_state)
            
            # Depending on current build mode change data source
            self.serial_data = self.mode_arbitration()
            
            # data = str(str(raw_data)).split("'")[1].split("\\")[0].split(",")
            
            # Display based on state
            if not self.fsm.is_idle:
                self.update_essential()
                self.update_widget6(self.serial_data[0:3])
                self.update_parameter_viewer(self.serial_data)
                
                if self.serial_data != None:
                    self.serialText.append(str(self.serial_data))
            else: 
                self.update_essential()
        
    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
            QtGui.QApplication.instance().exec_()
            
    def animation(self):
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(20)
        self.start()
        


if __name__ == '__main__':
    Window = MainWindow()
    Window.animation()
    

