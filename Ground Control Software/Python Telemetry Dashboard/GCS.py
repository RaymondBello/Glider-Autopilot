#!/usr/bin/python3

from datetime import datetime
from SERIAL import Comms
from TCP import WS_Manager
from UDP import UDP_Manager
from AVA import AVA
import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import QPushButton
import pyqtgraph.opengl as gl
import numpy as np
from math import *
import time
import sys
import json
import serial

class GCS_Plotter:
    '''This is the base class for the Ground Control Station Plotter'''
    
    v_serial = 0
    try:
        virtual_serial = serial.Serial('COM2')
        v_serial = 1
    except Exception as error :
        print(error)

    title = "Ground Control Station"
    background_color = (36, 37, 41)
    aspect_ratio_locked = True

    build_mode = None
    debug_mode = False
    tcp_mode = False
    serial_mode = False

    # State Constants
    ABORT_state = 0
    IDLE_state = 1
    LOGGING_STATE = 2
    ORIGIN_STATE = 3
    TAKE_OFF_state = 4
    RETURN_TO_HOME = 5

    # GUI WINDOW SIZE CONSTANTS
    WINDOW_WIDTH = 1100
    WINDOW_HEIGHT = 800

    # Button styles
    ButtonStyle_white = "background-color:rgb(255, 255, 255);color:rgb(0,0,0);font-size:26px;font-weight:bold"
    ButtonStyle_red = "background-color:rgb(200, 0, 0);color:rgb(0,0,0);font-size:26px;font-weight:bold"
    ButtonStyle_green = "background-color:rgb(0, 155, 0);color:rgb(0,0,0);font-size:26px;font-weight:bold"
    ButtonStyle_yellow = "background-color:rgb(255, 255, 0);color:rgb(0,0,0);font-size:26px;font-weight:bold"

    # Top Row Draw Buffers 
    data1_0 = [0] * 200
    data1_1 = [0] * 200
    data1_2 = [0] * 200
    
    data2_0 = [0] * 200
    data2_1 = [0] * 200
    data2_2 = [0] * 200
    
    data3_0 = [0] * 200
    data3_1 = [0] * 200
    data3_2 = [0] * 200    
    
    counter = 0

    def __init__(self):
        self.system_state_pool = (0, 1, 2, 3, 4, 5, 6)
        self.current_state = 1

        self.random_plot_step = 100
        self.raw_data = []
        self.serial_data = [1] * 19
        self.count = 0

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
        
        self.udp_socket = UDP_Manager()

        # Window Initialization
        self.win = pg.GraphicsWindow(size=(self.WINDOW_WIDTH, self.WINDOW_HEIGHT))
        self.win.setWindowTitle(self.title)
        self.win.setBackground(self.background_color)
        self.win.setAspectLocked(self.aspect_ratio_locked)

        # Font Initialization
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
        self.topSection = self.win.addLayout(colspan=5, rowspan=1)

        # Top Row Buttons
        self.proxy1 = QtGui.QGraphicsProxyWidget()
        button_abort_state = QtGui.QPushButton('ABORT')
        button_abort_state.setStyleSheet(self.ButtonStyle_red)
        button_abort_state.clicked.connect(self.abort_state)
        self.proxy1.setWidget(button_abort_state)
        self.topSection.addItem(self.proxy1)

        self.topSection.nextCol()

        self.proxy2 = QtGui.QGraphicsProxyWidget()
        button_data_log_start = QtGui.QPushButton('LOG TOGGLE')
        button_data_log_start.setStyleSheet(self.ButtonStyle_white)
        button_data_log_start.clicked.connect(self.armed_state)
        self.proxy2.setWidget(button_data_log_start)
        self.topSection.addItem(self.proxy2)

        self.topSection.nextCol()

        self.proxy3 = QtGui.QGraphicsProxyWidget()
        button_initialize = QtGui.QPushButton('SET ORIGIN')
        button_initialize.setStyleSheet(self.ButtonStyle_white)
        button_initialize.clicked.connect(self.idle_state)
        self.proxy3.setWidget(button_initialize)
        self.topSection.addItem(self.proxy3)

        self.topSection.nextCol()

        self.proxy4 = QtGui.QGraphicsProxyWidget()
        button_start = QtGui.QPushButton('START')
        button_start.setStyleSheet(self.ButtonStyle_green)
        button_start.clicked.connect(self.take_off_state)
        self.proxy4.setWidget(button_start)
        self.topSection.addItem(self.proxy4)

        self.topSection.nextCol()

        self.proxy5 = QtGui.QGraphicsProxyWidget()
        button_return_to_home = QtGui.QPushButton('RETURN')
        button_return_to_home.setStyleSheet(self.ButtonStyle_yellow)
        button_return_to_home.clicked.connect(self.return_home_state)
        self.proxy5.setWidget(button_return_to_home)
        self.topSection.addItem(self.proxy5)

        self.win.nextRow()
        
        # p2 = win.addPlot(title="Multiple curves")
        # p2.plot(np.random.normal(size=100), pen=(255,0,0), name="Red curve")
        # p2.plot(np.random.normal(size=110)+5, pen=(0,255,0), name="Green curve")
        # p2.plot(np.random.normal(size=120)+10, pen=(0,0,255), name="Blue curve")
        
        ''' Accelerometer Row '''
        self.graph1 = self.win.addPlot(title="Accelerometer")
        self.graph2 = self.win.addPlot(title="Gyroscope")
        self.graph3 = self.win.addPlot(title="Magnetometer")
        self.graph1.addLegend(offset=(1, 1))
        # self.graph2.addLegend(offset=(1, 1))
        # self.graph3.addLegend(offset=(1, 1))
        
        # Accelerometer
        self.graph1_curve = self.graph1.plot(
            self.data1_0,
            pen=(150, 0, 0),
            name="x",
            labels={
                "left": self.graph1.setLabel("left", text="Acceleration ", units="m/s^2"),
                # "bottom": self.graph1.setLabel("bottom", text="Time", units="s"),
            },
        )
        self.graph1_curve1 = self.graph1.plot(
            self.data1_1,
            pen=(0, 150, 0),
            name="y",
        )
        self.graph1_curve2 = self.graph1.plot(
            self.data1_2,
            pen=(0, 0, 150),
            name="z",
        )
       
        # Gyroscope
        self.graph2_curve = self.graph2.plot(
            self.data2_0,
            pen=(150, 0, 0),
            name="Gx",
            labels={
                "left": self.graph2.setLabel("left", text="Angular Accel ", units="rad/s2"),
                "bottom": self.graph1.setLabel("bottom", text="Time", units="s"),
            },
        )
        self.graph2_curve1 = self.graph2.plot(
            self.data2_1,
            pen=(0,150,0),
            name="Gy"
        )
        self.graph2_curve2 = self.graph2.plot(
            self.data2_2,
            pen=(0,0,150),
            name="Gz"
        )
        
        # Magnetometer
        self.graph3_curve = self.graph3.plot(
            self.data3_0,
            pen=(150, 0, 0),
            name="Mx",
            # labels={"bottom": graph3.setLabel("bottom", text="Time", units="s"),},
        )
        self.graph3_curve1 = self.graph3.plot(
            self.data3_1,
            pen=(0, 150, 0),
            name = "My"
        )
        self.graph3_curve2 = self.graph3.plot(
            self.data3_2,
            pen=(0, 0, 150),
            name = "Mz"
        )
        self.ptr1 = 0

        # State Flag Indicator
        self.StateGraphic = self.win.addPlot(title="Flag")
        self.StateGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.StateGraphic.hideAxis('bottom')
        self.StateGraphic.hideAxis('left')
        self.texttoState = pg.TextItem(f"IDLE", anchor=(0.5, 0.5), color='w')
        self.texttoState.setFont(self.font3)
        self.StateGraphic.addItem(self.texttoState)

        ''' Pitch Indicator '''
        self.PitchGraphic = self.win.addPlot(title="Pitch")
        self.PitchGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.PitchGraphic.hideAxis('bottom')
        self.PitchGraphic.hideAxis('left')
        self.texttoPitch = pg.TextItem("0.0°", anchor=(0.5, 0.5), color='w')
        self.texttoPitch.setFont(self.font3)
        self.PitchGraphic.addItem(self.texttoPitch)
        
        ''' RX Data Text Item'''
        self.RXDataGraphic = self.win.addPlot(title="Receiver Data ",colspan=1, rowspan=2)
        self.RXDataGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.RXDataGraphic.hideAxis('bottom')
        self.RXDataGraphic.hideAxis('left')
        self.texttoRX = pg.TextItem("null", anchor=(0.5, 0.5), color='w')
        self.texttoRX.setFont(self.font3)
        self.RXDataGraphic.addItem(self.texttoRX)
        

        self.win.nextRow()

        '''Pitch Roll Heading Row'''
        self.graph4 = self.win.addPlot(title="Pitch")
        self.graph5 = self.win.addPlot(title="Roll")
        self.graph6 = self.win.addPlot(title="Heading")
        # Used automatic downsampling and clipping to reduce the drawing load
        self.graph4.setDownsampling(mode="peak")
        self.graph5.setDownsampling(mode="peak")
        self.graph6.setDownsampling(mode="peak")
        self.graph4.setClipToView(True)
        self.graph5.setClipToView(True)
        self.graph6.setClipToView(True)
        
        self.graph4.addLegend(offset=(1, 1))
        
        self.graph4.setRange(xRange=[-200, 0])
        self.graph4.setLimits(xMax=0)
        self.graph4_curve = self.graph4.plot(
            pen=(150, 150, 75),
            name="Actual",
            labels={
                "left": self.graph4.setLabel("left", text="Angle", units="°"),
                # "bottom": self.graph4.setLabel("bottom", text="Time", units="s"),
            },
        )
        self.graph4_curve1 = self.graph4.plot(
            pen=(150, 0, 0),
            name="Setpoint",
        )
        
        self.graph5.setRange(xRange=[-200, 0])
        self.graph5.setLimits(xMax=0)
        self.graph5_curve = self.graph5.plot(
            pen=(150, 150, 75),
            name="Act",
            # labels={"bottom": self.graph5.setLabel("bottom", text="Time", units="s"),},
        )
        self.graph5_curve1 = self.graph5.plot(
            pen=(150, 0, 0),
            name="Setpoint",
        )
        
        self.graph6.setRange(xRange=[-200, 0])
        self.graph6.setLimits(xMax=0)
        self.graph6_curve = self.graph6.plot(
            pen=(150, 150, 75),
            name="Act",
            # labels={"bottom": self.graph6.setLabel("bottom", text="Time", units="s"),},
        )
        self.graph6_curve1 = self.graph6.plot(
            pen=(150, 0, 0),
            name="Setpoint",
        )
        
        self.data4 = np.empty(200)
        self.data4_1 = np.empty(200)
        self.data5 = np.empty(200)
        self.data5_1 = np.empty(200)
        self.data6 = np.empty(200)
        self.data6_1 = np.empty(200)
        self.ptr2 = 0

        ''' Battery Text Item '''
        self.BatteryGraphic = self.win.addPlot(title="Battery Voltage ")
        self.BatteryGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.BatteryGraphic.hideAxis('bottom')
        self.BatteryGraphic.hideAxis('left')
        self.texttoBattery = pg.TextItem(f"{11.2} V", anchor=(0.5, 0.5), color='w')
        self.texttoBattery.setFont(self.font2)
        self.BatteryGraphic.addItem(self.texttoBattery)

        ''' Roll Indicator '''
        self.RollGraphic = self.win.addPlot(title="Roll")
        self.RollGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.RollGraphic.hideAxis('bottom')
        self.RollGraphic.hideAxis('left')
        self.texttoRoll = pg.TextItem("0.0°", anchor=(0.5, 0.5), color='w')
        self.texttoRoll.setFont(self.font3)
        self.RollGraphic.addItem(self.texttoRoll)

        self.win.nextRow()

        self.win.nextRow()

        '''Magnetometer Row'''
        self.graph8 = self.win.addPlot(title="Ground Speed")
        self.graph9 = self.win.addPlot(title="Climb Rate")
        self.graph10 = self.win.addPlot(title="Altitude")
        self.graph8.setClipToView(True)
        self.graph9.setClipToView(True)
        self.graph10.setClipToView(True)

        self.graph8.setRange(xRange=[-200, 0])
        self.graph8.setLimits(xMax=0)
        self.graph8_curve = self.graph8.plot(
            pen=(150, 0, 0),
            name="Vel",
            labels={
                "left": self.graph8.setLabel("left", text="Velocity", units="m/s"),
                # "bottom": self.graph8.setLabel("bottom", text="Time", units="s"),
            },
        )
        self.graph9.setRange(xRange=[-200, 0])
        self.graph9.setLimits(xMax=0)
        self.graph9_curve = self.graph9.plot(
            pen=(0, 150, 0),
            name="Mag_Y",
            # labels={"bottom": self.graph9.setLabel("bottom", text="Time", units="s"),},
        )
        self.graph10.setRange(xRange=[-200, 0])
        self.graph10.setLimits(xMax=0)
        self.graph10_curve = self.graph10.plot(
            pen=(0, 0, 150),
            name="Mag_Z",
            labels={"left": self.graph10.setLabel("left", text="Meters", units="m"),},
        )
        self.data8 = np.empty(200)
        self.data9 = np.empty(200)
        self.data10 = np.empty(200)

        self.ptr4 = 0

        '''Pressure Graphic'''
        self.PressureGraphic = self.win.addPlot(title="Atm. Pressure")
        self.PressureGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.PressureGraphic.hideAxis('bottom')
        self.PressureGraphic.hideAxis('left')
        self.texttoPressure = pg.TextItem("1000 Pa", anchor=(0.5, 0.5), color='w')
        self.texttoPressure.setFont(self.font2)
        self.PressureGraphic.addItem(self.texttoPressure)

        ''' Heading Indicator '''
        self.HeadingGraphic = self.win.addPlot(title="Heading")
        self.HeadingGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.HeadingGraphic.hideAxis('bottom')
        self.HeadingGraphic.hideAxis('left')
        self.texttoHeading = pg.TextItem("0.0°", anchor=(0.5, 0.5), color='w')
        self.texttoHeading.setFont(self.font3)
        self.HeadingGraphic.addItem(self.texttoHeading)
        
        ''' Actuator Data Text Item'''
        self.ACTDataGraphic = self.win.addPlot(title="Actuator Data ",colspan=1, rowspan=2)
        self.ACTDataGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.ACTDataGraphic.hideAxis('bottom')
        self.ACTDataGraphic.hideAxis('left')
        self.texttoACT = pg.TextItem("null", anchor=(0.5, 0.5), color='w')
        self.texttoACT.setFont(self.font3)
        self.ACTDataGraphic.addItem(self.texttoACT)

        self.win.nextRow()

        ''' Internal Temperature graph '''
        # Plot in chunks, adding one new plot curve for every 100 samples
        self.chunkSize = 100
        # Remove chunks after we have 10
        self.maxChunks = 10
        self.startTime = pg.ptime.time()
        self.graph7 = self.win.addPlot(colspan=1, title="Internal Temperature")
        # self.graph7.setLabel("bottom", "Time", "s")
        self.graph7.setXRange(-10, 0)
        self.curves = []
        self.data7 = np.empty((self.chunkSize + 1, 2))
        self.ptr3 = 0

        ''' Time Text Item '''
        self.TimeGraphic = self.win.addPlot(title="Time (H:M:S)")
        self.TimeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.TimeGraphic.hideAxis('bottom')
        self.TimeGraphic.hideAxis('left')
        now = datetime.now()
        self.texttoTime = pg.TextItem(f"{now.strftime('%H:%M:%S')}",
                                anchor=(0.5, 0.5), color='w')
        self.texttoTime.setFont(self.font3)
        self.TimeGraphic.addItem(self.texttoTime)

        ''' GPS Data Text Item'''
        self.GPSDataGraphic = self.win.addPlot(title="GPS Data ",colspan=2, rowspan=1)
        self.GPSDataGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.GPSDataGraphic.hideAxis('bottom')
        self.GPSDataGraphic.hideAxis('left')
        self.texttoGPS = pg.TextItem("null", anchor=(0.5, 0.5), color='w')
        self.texttoGPS.setFont(self.font3)
        self.GPSDataGraphic.addItem(self.texttoGPS)

        ''' Altitude Text Item '''
        self.AltitudeGraphic = self.win.addPlot(title="Altitude")
        self.AltitudeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
        self.AltitudeGraphic.hideAxis('bottom')
        self.AltitudeGraphic.hideAxis('left')
        self.texttoAltitude = pg.TextItem(f"0.01m", anchor=(0.5, 0.5), color='w')
        self.texttoAltitude.setFont(self.font2)
        self.AltitudeGraphic.addItem(self.texttoAltitude)

        self.win.nextCol()

    def update_acc_gyro_mag(self, row_values):
        """Update graph-> shifts data in the array one sample left, appends new value"""
        self.shift_array(self.data1_0)
        self.shift_array(self.data1_1)
        self.shift_array(self.data1_2)
        self.shift_array(self.data2_0)
        self.shift_array(self.data2_1)
        self.shift_array(self.data2_2)
        self.shift_array(self.data3_0)
        self.shift_array(self.data3_1)
        self.shift_array(self.data3_2)

        if self.ptr1 < self.random_plot_step:
            self.data1_0[-1] = np.random.normal()
            self.data1_1[-1] = np.random.normal()
            self.data1_2[-1] = np.random.normal()
            self.data2_0[-1] = np.random.normal()
            self.data2_1[-1] = np.random.normal()
            self.data2_2[-1] = np.random.normal()
            self.data3_0[-1] = np.random.normal()
            self.data3_1[-1] = np.random.normal()
            self.data3_2[-1] = np.random.normal()
        else:
            if len(row_values) == 9:
                try:
                    data1_0_value = round(float(row_values[0]), 3)
                    data1_1_value = round(float(row_values[1]), 3)
                    data1_2_value = round(float(row_values[2]), 3)
                    data2_0_value = round(float(row_values[3]), 3)
                    data2_1_value = round(float(row_values[4]), 3)
                    data2_2_value = round(float(row_values[5]), 3)
                    data3_0_value = round(float(row_values[6]), 3)
                    data3_1_value = round(float(row_values[7]), 3)
                    data3_2_value = round(float(row_values[8]), 3)

                    self.data1_0[-1] = data1_0_value
                    self.data1_1[-1] = data1_1_value
                    self.data1_2[-1] = data1_2_value
                    self.data2_0[-1] = data2_0_value
                    self.data2_1[-1] = data2_1_value
                    self.data2_2[-1] = data2_2_value
                    self.data3_0[-1] = data3_0_value
                    self.data3_1[-1] = data3_1_value
                    self.data3_2[-1] = data3_2_value
                except ValueError as error:
                    print(f"[VALUE ERROR]: {error}")
        
        

        self.ptr1 += 1
        self.graph1_curve.setData(self.data1_0)
        self.graph1_curve.setPos(self.ptr1, 0)
        self.graph1_curve1.setData(self.data1_1)
        self.graph1_curve1.setPos(self.ptr1, 0)
        self.graph1_curve2.setData(self.data1_2)
        self.graph1_curve2.setPos(self.ptr1, 0)
        
        self.graph2_curve.setData(self.data2_0)
        self.graph2_curve.setPos(self.ptr1, 0)
        self.graph2_curve1.setData(self.data2_1)
        self.graph2_curve1.setPos(self.ptr1, 0)
        self.graph2_curve2.setData(self.data2_2)
        self.graph2_curve2.setPos(self.ptr1, 0)
        
        self.graph3_curve.setData(self.data3_0)
        self.graph3_curve.setPos(self.ptr1, 0)
        self.graph3_curve1.setData(self.data3_1)
        self.graph3_curve1.setPos(self.ptr1, 0)
        self.graph3_curve2.setData(self.data3_2)
        self.graph3_curve2.setPos(self.ptr1, 0)

    def update_pitch_roll_yaw(self, pry_values):
        if self.ptr2 < self.random_plot_step:
            self.data4[self.ptr2] = np.random.normal()
            self.data4_1[self.ptr2] = np.random.normal()
            self.data5[self.ptr2] = np.random.normal()
            self.data5_1[self.ptr2] = np.random.normal()
            self.data6[self.ptr2] = np.random.normal()
            self.data6_1[self.ptr2] = np.random.normal()
            
        else:
            if len(pry_values) == 6:
                try:
                    data4_value = round(float(pry_values[0]), 3)
                    data4_1_value = round(float(pry_values[1]), 3)
                    data5_value = round(float(pry_values[2]), 3)
                    data5_1_value = round(float(pry_values[3]), 3)
                    data6_value = round(float(pry_values[4]), 3)
                    data6_1_value = round(float(pry_values[5]), 3)

                    self.data4[self.ptr2] = data4_value  # round(float(pry_values[0]), 3)
                    self.data4_1[self.ptr2] = data4_1_value
                    self.data5[self.ptr2] = data5_value
                    self.data5_1[self.ptr2] = data5_1_value
                    self.data6[self.ptr2] = data6_value
                    self.data6_1[self.ptr2] = data6_1_value
                except ValueError as error:
                    print(f"[VALUE ERROR]: {error}")

        self.ptr2 += 1

        if self.ptr2 >= self.data4.shape[0]:
            tmp1 = self.data4
            tmp1_1 = self.data4_1
            tmp2 = self.data5
            tmp2_1 = self.data5_1
            tmp3 = self.data6
            tmp3_1 = self.data6_1

            self.data4 = np.empty(self.data4.shape[0] * 2)
            self.data4_1 = np.empty(self.data4_1.shape[0] * 2)
            self.data5 = np.empty(self.data5.shape[0] * 2)
            self.data5_1 = np.empty(self.data5_1.shape[0] * 2)
            self.data6 = np.empty(self.data6.shape[0] * 2)
            self.data6_1 = np.empty(self.data6_1.shape[0] * 2)

            self.data4[: tmp1.shape[0]] = tmp1
            self.data4_1[: tmp1_1.shape[0]] = tmp1_1
            self.data5[: tmp2.shape[0]] = tmp2
            self.data5_1[: tmp2_1.shape[0]] = tmp2_1
            self.data6[: tmp3.shape[0]] = tmp3
            self.data6_1[: tmp3_1.shape[0]] = tmp3_1

        self.graph4_curve.setData(self.data4[:self.ptr2])
        self.graph4_curve1.setData(self.data4_1[:self.ptr2])
        self.graph5_curve.setData(self.data5[:self.ptr2])
        self.graph5_curve1.setData(self.data5_1[:self.ptr2])
        self.graph6_curve.setData(self.data6[:self.ptr2])
        self.graph6_curve1.setData(self.data6_1[:self.ptr2])

        self.graph4_curve.setPos(-self.ptr2, 0)
        self.graph4_curve1.setPos(-self.ptr2, 0)
        self.graph5_curve.setPos(-self.ptr2, 0)
        self.graph5_curve1.setPos(-self.ptr2, 0)
        self.graph6_curve.setPos(-self.ptr2, 0)
        self.graph6_curve1.setPos(-self.ptr2, 0)

    
    def update_pressure(self, pressure):
        self.PressureGraphic.removeItem(self.texttoPressure)
        self.texttoPressure = pg.TextItem(f"{str(round(float(pressure[0]),1))} Pa", anchor=(0.5, 0.5), color='w')
        self.texttoPressure.setFont(self.font3)
        self.PressureGraphic.addItem(self.texttoPressure)
    
    def update_vel_climb_alt(self, mag_values):
        if self.ptr4 < self.random_plot_step:
            self.data8[self.ptr4] = np.random.normal()
            self.data9[self.ptr4] = np.random.normal()
            self.data10[self.ptr4] = np.random.normal()
        else:
            if len(mag_values) == 3:
                try:
                    data8_value = round(float(mag_values[0]), 3)
                    data9_value = round(float(mag_values[1]), 3)
                    data10_value = round(float(mag_values[2]), 3)

                    self.data8[self.ptr4] = data8_value
                    self.data9[self.ptr4] = data9_value
                    self.data10[self.ptr4] = data10_value
                    
                except ValueError as error:
                    print(f"[VALUE ERROR]: {error}")

        self.ptr4 += 1
        
        if self.ptr4 >= self.data8.shape[0]:
            temp1 = self.data8
            temp2 = self.data9
            temp3 = self.data10

            self.data8 = np.empty(self.data8.shape[0] * 2)
            self.data9 = np.empty(self.data9.shape[0] * 2)
            self.data10 = np.empty(self.data10.shape[0] * 2)
            
            self.data8[: temp1.shape[0]] = temp1
            self.data9[: temp2.shape[0]] = temp2
            self.data10[: temp3.shape[0]] = temp3

        self.graph8_curve.setData(self.data8[:self.ptr4])
        self.graph9_curve.setData(self.data9[:self.ptr4])
        self.graph10_curve.setData(self.data10[:self.ptr4])

        self.graph8_curve.setPos(-self.ptr4, 0)
        self.graph9_curve.setPos(-self.ptr4, 0)
        self.graph10_curve.setPos(-self.ptr4, 0)
    
    def update_row_temp(self, temp_values):
        now = pg.ptime.time()
        for c in self.curves:
            c.setPos(-(now - self.startTime), 0)

        i = self.ptr3 % self.chunkSize
        if i == 0:
            self.graph7_curve = self.graph7.plot(
                pen=(148, 0, 211),
                name="Temp",
                labels={
                    "left": self.graph7.setLabel("left", text="Celsius", units="°C"),
                    "bottom": self.graph7.setLabel("bottom", text="Time", units="s"),
                },
            )
            self.curves.append(self.graph7_curve)
            last = self.data7[-1]
            self.data7 = np.empty((self.chunkSize + 1, 2))
            self.data7[0] = last
            while len(self.curves) > self.maxChunks:
                c = self.curves.pop(0)
                self.graph7.removeItem(c)
        else:
            self.graph7_curve = self.curves[-1]

        self.data7[i + 1, 0] = now - self.startTime

        if len(temp_values) == 1:
            try:
                data7_value = round(float(temp_values[0]), 3)
                self.data7[i + 1, 1] = data7_value
            except ValueError as error:
                print(f"[VALUE ERROR]: {error}")

        self.graph7_curve.setData(x=self.data7[: i + 2, 0], y=self.data7[: i + 2, 1])
        self.ptr3 += 1

    def update_altitude(self, altitude):
        self.AltitudeGraphic.removeItem(self.texttoAltitude)
        self.texttoAltitude = pg.TextItem(f"{str(round(float(altitude[0]),2))} m", anchor=(0.5, 0.5), color='w')
        self.texttoAltitude.setFont(self.font3)
        self.AltitudeGraphic.addItem(self.texttoAltitude)
    
    def update_time(self):
        self.TimeGraphic.removeItem(self.texttoTime)
        now = datetime.now()
        self.texttoTime = pg.TextItem(f"{now.strftime('%H:%M:%S')}", anchor=(0.5, 0.5), color="w")
        self.texttoTime.setFont(self.font3)
        self.TimeGraphic.addItem(self.texttoTime)

    @staticmethod
    def GPS_text_template(values: list):
        gps_temp = f"Lat: {values[0]}\tLng: {values[1]}\n# of Sat: {values[2]}\tSignal Str: {82}%\nAlt: {102} m\tSpeed: {0.8} m/s\nHeading: {310}°\tUpdated: {5} secs ago"
        return pg.TextItem(gps_temp,anchor=(0.5, 0.5), color='w')
    
    @staticmethod
    def RX_text_template(values:list):
        values = [i*1000 for i in values] # REMOVE THIS LINE
        
        rx_temp = f"RX_CH1: {round(int(values[2]),0)}\nRX_CH2: {round(int(values[2]),0)}\nRX_CH3: {round(int(values[2]),0)}\nRX_CH4: {round(int(values[2]),0)}\nRX_CH5: {round(int(values[2]),0)}\nRX_CH6: {round(int(values[2]),0)}\nRX_CH7: {round(int(values[2]),0)}\nRX_CH8: {round(int(values[2]),0)}\nRX_CH9: {round(int(values[2]),0)}\nRX_CH10: {round(int(values[2]),0)}"
        return pg.TextItem(rx_temp,anchor=(0.5, 0.5), color='w')
    
    @staticmethod
    def ACT_text_template(values: list):
        values = [i*1000 for i in values] # REMOVE THIS LINE
        
        act_temp = f"ACT_CH1: {round(int(values[2]),0)}\nACT_CH2: {round(int(values[2]),0)}\nACT_CH3: {round(int(values[2]),0)}\nACT_CH4: {round(int(values[2]),0)}\nACT_CH5: {round(int(values[2]),0)}\nAIL: {round(int(values[2]),0)}°\nELE: {round(int(values[2]),0)}°\nRUD: {round(int(values[2]),0)}°\nTHR: {round(int(values[2]),0)}%\n"
        return pg.TextItem(act_temp,anchor=(0.5, 0.5), color='w')
        
    
    def update_gps(self, gps_values):
        self.GPSDataGraphic.removeItem(self.texttoGPS)
        self.texttoGPS = self.GPS_text_template(gps_values)
        self.texttoGPS.setFont(self.font_gps)
        self.GPSDataGraphic.addItem(self.texttoGPS)
    
    def update_rx(self, rx_values):
        self.RXDataGraphic.removeItem(self.texttoRX)
        self.texttoRX = self.RX_text_template(rx_values)
        self.texttoRX.setFont(self.font_gps)
        self.RXDataGraphic.addItem(self.texttoRX)
    
    def update_actuator(self, act_values):
        self.ACTDataGraphic.removeItem(self.texttoACT)
        self.texttoACT = self.ACT_text_template(act_values)
        self.texttoACT.setFont(self.font_gps)
        self.ACTDataGraphic.addItem(self.texttoACT)

    def update_battery(self,bat_val):
        self.BatteryGraphic.removeItem(self.texttoBattery)
        self.texttoBattery = pg.TextItem(f"{bat_val} V", anchor=(0.5, 0.5), color='w')
        self.texttoBattery.setFont(self.font3)
        self.BatteryGraphic.addItem(self.texttoBattery)

    def write_json(self, data, filename="web/sensor_data.json"):
        with open(filename, "w") as f:
            json.dump(data, f, indent=3)

    def write_data_to_JSON(self, jcontent):
        with open("web/sensor_data.json") as json_file:
            data = json.load(json_file)
            temp = data["packet"]
            temp[4]["data"]["Pressure"] = float(jcontent[13])
            temp[4]["data"]["Altitude"] = float(jcontent[14])
            temp[4]["data"]["Temperature"] = float(jcontent[15])
            temp[5]["data"]["Pitch"] = float(jcontent[16])
            temp[5]["data"]["Roll"] = float(jcontent[17])
            temp[5]["data"]["Yaw"] = float(jcontent[18])

        # print(jcontent)
        self.write_json(data)

    @staticmethod
    def ABORT():
        ''' State Switch Case Function '''
        return "ABORT"

    @staticmethod
    def IDLE():
        ''' State Switch Case Function '''
        return "IDLE"

    @staticmethod
    def ARMED():
        ''' State Switch Case Function '''
        return "LOGGING"

    @staticmethod
    def ORIGIN():
        ''' State Switch Case Function '''
        return "HOME SET"

    @staticmethod
    def TAKEOFF():
        ''' State Switch Case Function '''
        return "TAKE-OFF"

    def update_state(self):
        '''Update the state text and variable'''
        STATE = int(self.current_state)
        options = {
            0: self.ABORT,
            1: self.IDLE,
            2: self.ARMED,
            3: self.ORIGIN,
            4: self.TAKEOFF,
        }
        state_string = options[STATE]()
        self.StateGraphic.removeItem(self.texttoState)
        self.texttoState = pg.TextItem(str(state_string), anchor=(0.5, 0.5), color='w')
        self.texttoState.setFont(self.font3)
        self.StateGraphic.addItem(self.texttoState)

    def shift_array(self, array):
        array[:-1] = array[1:]
    
    def tcp_handleshack(self, commandType: str, commandData: str):
        ''' Takes the command type and command data and returns raw data'''
        self.ws_socket.send_data(commandType, commandData)
        self.raw_data = self.ws_socket.receive_data()

        return self.raw_data

    def udp_broadcast(self, data):
        ''' Send udp packet to telemetry viewer'''
        self.udp_socket.send_data(data)
        return True

    def abort_state(self):
        '''abort function'''
        self.current_state = int(not bool(self.current_state))
        if self.tcp_mode:
            self.raw_data = self.tcp_handleshack("STATE", self.current_state)
    
    def idle_state(self):
        if self.current_state == self.IDLE_state:
            self.current_state = self.ORIGIN_STATE
        if self.current_state == self.ABORT_state:
            self.current_state = self.IDLE_state
        if self.current_state == self.ORIGIN_STATE:
            self.current_state = self.ORIGIN_STATE
        if self.current_state == self.LOGGING_STATE:
            self.current_state = self.ORIGIN_STATE

    def armed_state(self):
        if self.current_state == self.IDLE_state:
            self.current_state = self.LOGGING_STATE
        else:
            self.current_state = self.IDLE_state
    
    def take_off_state(self):
        if self.current_state == self.TAKE_OFF_state:
            self.current_state = self.IDLE_state
        if self.current_state == self.ABORT_state:
            self.current_state = self.IDLE_state
        if self.current_state == self.ORIGIN_STATE:
            self.current_state = self.TAKE_OFF_state
        if self.current_state == self.LOGGING_STATE:
            self.current_state = self.TAKE_OFF_state
    
    def return_home_state(self):
        pass
    
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
        
        return [2*sin(self.counter + (idx*180 if bool(idx%2) else -idx*90) ) for idx, val in enumerate(self.serial_data)]
        

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

    def start(self):
        ''' Start Qt event loop unless running in interactive mode'''
        if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
            QtGui.QApplication.instance().exec_()

    def send_to_serial(self):
        if self.v_serial:
            data_string = ""
            
            for idx,val in enumerate(self.serial_data):
                data_string += str(val)
                if idx == len(self.serial_data)-1:
                    data_string += "\n"
                else:
                    data_string += ","
                    
            self.virtual_serial.write(data_string.encode('utf-8'))
        
    def update(self):
        ''' Updates all UI data'''
        start = time.perf_counter()

        # Depending on current build mode
        if self.tcp_mode:
            self.serial_data = self.tcp_handleshack("STATE", self.current_state)
            self.serial_data = self.AVA_model.update(self.serial_data, self.current_state)
            self.udp_broadcast(self.serial_data)
            
        if self.debug_mode:
            self.serial_data = self.generate_data(self.serial_data)
            # print(test, len(test))
            
        if self.serial_mode:
            print("Serial Mode")
            
        self.update_state()

        if self.current_state in [self.IDLE_state, self.LOGGING_STATE, self.ORIGIN_STATE, self.TAKE_OFF_state]:
            self.update_acc_gyro_mag(self.serial_data[0:9])
            self.update_pitch_roll_yaw(self.serial_data[3:9])
            # self.update_rx_act(self.serial_data[3:9])
            self.update_vel_climb_alt(self.serial_data[6:9])
            self.update_pressure(self.serial_data[13:14])
            self.update_altitude(self.serial_data[14:15])
            self.update_row_temp(self.serial_data[0:1])
            self.update_time()
            self.update_gps([45.4534, 72.3424, 5])
            self.update_rx(self.serial_data[3:9])
            self.update_actuator(self.serial_data[3:9])
            self.update_battery(12)
            self.write_data_to_JSON(self.serial_data)
        if self.current_state in [self.ABORT_state]:
            self.update_time()
            self.update_gps([45.4534, 72.3424, 5])
            self.update_battery(12)

        finish = time.perf_counter()
        # print(f"[GRAPH] Update Rate: {round(time.perf_counter()-start,5)} seconds ")
        
        '''Edit this'''
        # self.send_to_serial()

    def animation(self):
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(5)
        self.start()

if __name__ == "__main__":
    Plot = GCS_Plotter.from_argv()
    Plot.animation()
