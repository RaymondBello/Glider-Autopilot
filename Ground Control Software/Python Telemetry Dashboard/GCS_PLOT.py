#!/usr/bin/python3

from datetime import datetime
from communication import Comms
from TCP import TCP_Manager
from AVA import AVA
import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import QPushButton
import pyqtgraph.opengl as gl
import numpy as np
import time
import sys
import json

if sys.argv[-1].lower() in ["true", "t"]:
    USE_TCP = True
else:
    # Boolean Flag for whether to use TCP Socket from class TCP_Manager in 'TCP.py'
    USE_TCP = False
      
RANDOM_PLOT_STEP = 50     # The number of steps random data will be plotted on GUI start-up. Change to 1 if unwanted.

# Serial connection is setup here
# comms = Comms()

# TCP connection is setup here
if USE_TCP:
    print("[SET-UP] : Setting up TCP connection")
    try:
        tcp_socket = TCP_Manager()
        print("[SET-UP] : TCP Connection Establied!")
    except Exception as identifier:
        print(identifier)

# State Management
SYSTEM_STATE_POOL = (0, 1, 2, 3, 4, 5, 6)
CURRENT_STATE = 1

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

data1 = [0] * 200
data2 = [0] * 200
data3 = [0] * 200

win = pg.GraphicsWindow(size=(WINDOW_WIDTH, WINDOW_HEIGHT))
win.setWindowTitle("GNC Telemetry Data")
win.setBackground((36, 37, 41))
win.setAspectLocked(True)

# Font
font1 = QtGui.QFont()
font1.setPixelSize(50)
font1.setWeight(100)

font2 = QtGui.QFont()
font2.setPixelSize(35)
font2.setWeight(100)

# buttons style
ButtonStyle_white = "background-color:rgb(255, 255, 255);color:rgb(0,0,0);font-size:26px;font-weight:bold"
ButtonStyle_red = "background-color:rgb(200, 0, 0);color:rgb(0,0,0);font-size:26px;font-weight:bold"
ButtonStyle_green = "background-color:rgb(0, 155, 0);color:rgb(0,0,0);font-size:26px;font-weight:bold"
ButtonStyle_yellow = "background-color:rgb(255, 255, 0);color:rgb(0,0,0);font-size:26px;font-weight:bold"

topSection = win.addLayout(colspan=4,rowspan=1)

def abort_state():
    global CURRENT_STATE
    CURRENT_STATE = int(not bool(CURRENT_STATE))
    if USE_TCP:
        tcp_handleshack("STATE",CURRENT_STATE)

def idle_state():
    global CURRENT_STATE
    if CURRENT_STATE == IDLE_state:
        CURRENT_STATE = ORIGIN_STATE
    if CURRENT_STATE == ABORT_state:
        CURRENT_STATE = IDLE_state
    if CURRENT_STATE == ORIGIN_STATE:
        CURRENT_STATE = ORIGIN_STATE
    if CURRENT_STATE == LOGGING_STATE:
        CURRENT_STATE = ORIGIN_STATE

def armed_state():
    global CURRENT_STATE
    if CURRENT_STATE == IDLE_state:
        CURRENT_STATE = LOGGING_STATE
    else:
        CURRENT_STATE = IDLE_state

def take_off_state():
    global CURRENT_STATE
    if CURRENT_STATE == TAKE_OFF_state:
        CURRENT_STATE = IDLE_state
    if CURRENT_STATE == ABORT_state:
        CURRENT_STATE = IDLE_state
    if CURRENT_STATE == ORIGIN_STATE:
        CURRENT_STATE = TAKE_OFF_state
    if CURRENT_STATE == LOGGING_STATE:
        CURRENT_STATE = TAKE_OFF_state

def return_home_state():
    pass

proxy3 = QtGui.QGraphicsProxyWidget()
button_abort_state = QtGui.QPushButton('ABORT')
button_abort_state.setStyleSheet(ButtonStyle_red)
button_abort_state.clicked.connect(abort_state)
proxy3.setWidget(button_abort_state)
topSection.addItem(proxy3)

topSection.nextCol()

proxy2 = QtGui.QGraphicsProxyWidget()
button_data_log_start = QtGui.QPushButton('LOG TOGGLE')
button_data_log_start.setStyleSheet(ButtonStyle_white)
button_data_log_start.clicked.connect(armed_state)
proxy2.setWidget(button_data_log_start)
topSection.addItem(proxy2)

topSection.nextCol()

proxy = QtGui.QGraphicsProxyWidget()
button_initialize = QtGui.QPushButton('SET ORIGIN')
button_initialize.setStyleSheet(ButtonStyle_white)
button_initialize.clicked.connect(idle_state)
proxy.setWidget(button_initialize)
topSection.addItem(proxy)

topSection.nextCol()

proxy4 = QtGui.QGraphicsProxyWidget()
button_start = QtGui.QPushButton('START')
button_start.setStyleSheet(ButtonStyle_green)
button_start.clicked.connect(take_off_state)
proxy4.setWidget(button_start)
topSection.addItem(proxy4)

topSection.nextCol()

proxy5 = QtGui.QGraphicsProxyWidget()
button_return_to_home = QtGui.QPushButton('RETURN')
button_return_to_home.setStyleSheet(ButtonStyle_yellow)
button_return_to_home.clicked.connect(return_home_state)
proxy5.setWidget(button_return_to_home)
topSection.addItem(proxy5)

win.nextRow()



graph1 = win.addPlot(title="Accelerometer X")
graph2 = win.addPlot(title="Accelerometer Y")
graph3 = win.addPlot(title="Accelerometer Z")
# graph1.addLegend(offset=(1, 1))
# graph2.addLegend(offset=(1, 1))
# graph3.addLegend(offset=(1, 1))

graph1_curve = graph1.plot(
    data1,
    pen=(150, 0, 0),
    name="Accel_X",
    labels={
        "left": graph1.setLabel("left", text="Acceleration ", units="m/s^2"),
        # "bottom": graph1.setLabel("bottom", text="Time", units="s"),
    },
)
graph2_curve = graph2.plot(
    data2,
    pen=(0, 150, 0),
    name="Accel_Y",
    # labels={"bottom": graph2.setLabel("bottom", text="Time", units="s"),},
)
graph3_curve = graph3.plot(
    data3,
    pen=(0, 0, 150),
    name="Accel_Z",
    # labels={"bottom": graph3.setLabel("bottom", text="Time", units="s"),},
)
ptr1 = 0


def shift_array(array):
    array[:-1] = array[1:]

def update_row_accel(acc_values):
    """Update graph-> shifts data in the array one sample left, appends new value"""

    global data1, data2, data3, graph1_curve, graph2_curve, graph3_curve, ptr1

    shift_array(data1)
    shift_array(data2)
    shift_array(data3)
    # data2[:-1] = data2[1:]

    if ptr1 < RANDOM_PLOT_STEP:
        data1[-1] = np.random.normal()
        data2[-1] = np.random.normal()
        data3[-1] = np.random.normal()

    else:
        if len(acc_values) == 3:
            try:
                data1_value = round(float(acc_values[0]), 3)
                data2_value = round(float(acc_values[1]), 3)
                data3_value = round(float(acc_values[2]), 3)

                data1[-1] = data1_value  # int(float(acc_values[2]))
                data2[-1] = data2_value
                data3[-1] = data3_value
            except ValueError as error:
                print(f"[VALUE ERROR]: {error}")

    ptr1 += 1
    graph1_curve.setData(data1)
    graph1_curve.setPos(ptr1, 0)

    graph2_curve.setData(data2)
    graph2_curve.setPos(ptr1, 0)

    graph3_curve.setData(data3)
    graph3_curve.setPos(ptr1, 0)

StateGraphic = win.addPlot(title="Flag")
StateGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
StateGraphic.hideAxis('bottom')
StateGraphic.hideAxis('left')
texttoState = pg.TextItem(f"IDLE_state", anchor=(0.5, 0.5), color='w')
texttoState.setFont(font2)
StateGraphic.addItem(texttoState)

win.nextRow()

'''Gyroscope Row'''

graph4 = win.addPlot(title="Gyroscope X")
graph5 = win.addPlot(title="Gyroscope Y")
graph6 = win.addPlot(title="Gyroscope Z")

# Use automatic downsampling and clipping to reduce the drawing load
graph4.setDownsampling(mode="peak")
graph5.setDownsampling(mode="peak")
graph6.setDownsampling(mode="peak")

graph4.setClipToView(True)
graph5.setClipToView(True)
graph6.setClipToView(True)

graph4.setRange(xRange=[-200, 0])
graph4.setLimits(xMax=0)
graph4_curve = graph4.plot(
    pen=(150, 0, 0),
    name="Gyro_X",
    labels={
        "left": graph4.setLabel("left", text="Angular Velocity", units="°/s"),
        # "bottom": graph4.setLabel("bottom", text="Time", units="s"),
    },
)

graph5.setRange(xRange=[-200, 0])
graph5.setLimits(xMax=0)
graph5_curve = graph5.plot(
    pen=(0, 150, 0),
    name="Gyro_Y",
    # labels={"bottom": graph5.setLabel("bottom", text="Time", units="s"),},
)

graph6.setRange(xRange=[-200, 0])
graph6.setLimits(xMax=0)
graph6_curve = graph6.plot(
    pen=(0, 0, 150),
    name="Gyro_Z",
    # labels={"bottom": graph6.setLabel("bottom", text="Time", units="s"),},
)

data4 = np.empty(200)
data5 = np.empty(200)
data6 = np.empty(200)

ptr2 = 0

BatteryGraphic = win.addPlot(title="Battery Voltage ")
BatteryGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
BatteryGraphic.hideAxis('bottom')
BatteryGraphic.hideAxis('left')
texttoBattery = pg.TextItem(f"{11.2} V", anchor=(0.5, 0.5), color='w')
texttoBattery.setFont(font2)
BatteryGraphic.addItem(texttoBattery)


def update_row_gyro(gyro_values):
    """Plot in chunks, adding one new plot curve for every 100 samples"""

    global data4, data5, data6, ptr2

    if ptr2 < RANDOM_PLOT_STEP:
        data4[ptr2] = np.random.normal()
        data5[ptr2] = np.random.normal()
        data6[ptr2] = np.random.normal()
    else:
        if len(gyro_values) == 3:
            try:
                data4_value = round(float(gyro_values[0]), 3)
                data5_value = round(float(gyro_values[1]), 3)
                data6_value = round(float(gyro_values[2]), 3)

                data4[ptr2] = data4_value  # round(float(gyro_values[0]), 3)
                data5[ptr2] = data5_value
                data6[ptr2] = data6_value
            except ValueError as error:
                print(f"[VALUE ERROR]: {error}")

    ptr2 += 1

    if ptr2 >= data4.shape[0]:
        tmp1 = data4
        tmp2 = data5
        tmp3 = data6

        data4 = np.empty(data4.shape[0] * 2)
        data5 = np.empty(data5.shape[0] * 2)
        data6 = np.empty(data6.shape[0] * 2)

        data4[: tmp1.shape[0]] = tmp1
        data5[: tmp2.shape[0]] = tmp2
        data6[: tmp3.shape[0]] = tmp3

    graph4_curve.setData(data4[:ptr2])
    graph5_curve.setData(data5[:ptr2])
    graph6_curve.setData(data6[:ptr2])

    graph4_curve.setPos(-ptr2, 0)
    graph5_curve.setPos(-ptr2, 0)
    graph6_curve.setPos(-ptr2, 0)

win.nextRow()

'''Magnetometer Row'''

graph8 = win.addPlot(title="Magnetometer X")
graph9 = win.addPlot(title="Magnetometer Y")
graph10 = win.addPlot(title="Magnetometer Z")

graph8.setClipToView(True)
graph9.setClipToView(True)
graph10.setClipToView(True)

graph8.setRange(xRange=[-200, 0])
graph8.setLimits(xMax=0)
graph8_curve = graph8.plot(
    pen=(150, 0, 0),
    name="Mag_X",
    labels={
        "left": graph8.setLabel("left", text="Magnetic Field Strength", units="μT"),
        # "bottom": graph8.setLabel("bottom", text="Time", units="s"),
    },
)

graph9.setRange(xRange=[-200, 0])
graph9.setLimits(xMax=0)
graph9_curve = graph9.plot(
    pen=(0, 150, 0),
    name="Mag_Y",
    # labels={"bottom": graph9.setLabel("bottom", text="Time", units="s"),},
)

graph10.setRange(xRange=[-200, 0])
graph10.setLimits(xMax=0)
graph10_curve = graph10.plot(
    pen=(0, 0, 150),
    name="Mag_Z",
    # labels={"bottom": graph10.setLabel("bottom", text="Time", units="s"),},
)

data8 = np.empty(200)
data9 = np.empty(200)
data10 = np.empty(200)

ptr4 = 0

'''Space for Text Field'''
PressureGraphic = win.addPlot(title="Atm. Pressure")
PressureGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
PressureGraphic.hideAxis('bottom')
PressureGraphic.hideAxis('left')
texttoPressure = pg.TextItem(f"{1000} Pa", anchor=(0.5, 0.5), color='w')
texttoPressure.setFont(font2)
PressureGraphic.addItem(texttoPressure)

def update_pressure(pressure):
    global texttoPressure

    PressureGraphic.removeItem(texttoPressure)
    texttoPressure = pg.TextItem(f"{str(round(float(pressure[0]),1))} Pa", anchor=(0.5, 0.5), color='w')
    texttoPressure.setFont(font2)
    PressureGraphic.addItem(texttoPressure)

def update_row_mag(mag_values):
    """Plot in chunks, adding one new plot curve for every 100 samples"""
    global data8, data9, data10, ptr4

    if ptr4 < RANDOM_PLOT_STEP:
        data8[ptr4] = np.random.normal()
        data9[ptr4] = np.random.normal()
        data10[ptr4] = np.random.normal()
    else:
        if len(mag_values) == 3:
            try:
                data8_value = round(float(mag_values[0]), 3)
                data9_value = round(float(mag_values[1]), 3)
                data10_value = round(float(mag_values[2]), 3)

                data8[ptr4] = data8_value
                data9[ptr4] = data9_value
                data10[ptr4] = data10_value
                
            except ValueError as error:
                print(f"[VALUE ERROR]: {error}")

    ptr4 += 1
    
    if ptr4 >= data8.shape[0]:
        temp1 = data8
        temp2 = data9
        temp3 = data10

        data8 = np.empty(data8.shape[0] * 2)
        data9 = np.empty(data9.shape[0] * 2)
        data10 = np.empty(data10.shape[0] * 2)
        
        data8[: temp1.shape[0]] = temp1
        data9[: temp2.shape[0]] = temp2
        data10[: temp3.shape[0]] = temp3

    graph8_curve.setData(data8[:ptr4])
    graph9_curve.setData(data9[:ptr4])
    graph10_curve.setData(data10[:ptr4])

    graph8_curve.setPos(-ptr4, 0)
    graph9_curve.setPos(-ptr4, 0)
    graph10_curve.setPos(-ptr4, 0)
        
win.nextRow()

# Plot in chunks, adding one new plot curve for every 100 samples
chunkSize = 100

# Remove chunks after we have 10
maxChunks = 10

startTime = pg.ptime.time()

graph7 = win.addPlot(colspan=1, title="Internal Temperature")
# graph7.setLabel("bottom", "Time", "s")
graph7.setXRange(-10, 0)

curves = []
data7 = np.empty((chunkSize + 1, 2))
ptr3 = 0

def update_row_temp(temp_values):
    global graph7, data7, ptr3, curves
    now = pg.ptime.time()
    for c in curves:
        c.setPos(-(now - startTime), 0)

    i = ptr3 % chunkSize
    if i == 0:
        graph7_curve = graph7.plot(
            pen=(148, 0, 211),
            name="Temp",
            labels={
                "left": graph7.setLabel("left", text="Celsius", units="°C"),
                "bottom": graph7.setLabel("bottom", text="Time", units="s"),
            },
        )
        curves.append(graph7_curve)
        last = data7[-1]
        data7 = np.empty((chunkSize + 1, 2))
        data7[0] = last
        while len(curves) > maxChunks:
            c = curves.pop(0)
            graph7.removeItem(c)
    else:
        graph7_curve = curves[-1]

    data7[i + 1, 0] = now - startTime

    if len(temp_values) == 1:
        try:
            data7_value = round(float(temp_values[0]), 3)
            data7[i + 1, 1] = data7_value
        except ValueError as error:
            print(f"[VALUE ERROR]: {error}")

    graph7_curve.setData(x=data7[: i + 2, 0], y=data7[: i + 2, 1])
    ptr3 += 1

TimeGraphic = win.addPlot(title="Time (H:M:S)")
TimeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))

TimeGraphic.hideAxis('bottom')
TimeGraphic.hideAxis('left')
now = datetime.now()
texttoTime = pg.TextItem(f"{now.strftime('%H:%M:%S')}",
                         anchor=(0.5, 0.5), color='w')
texttoTime.setFont(font1)
TimeGraphic.addItem(texttoTime)

GPSDataGraphic = win.addPlot(title="GPS Data ")
GPSDataGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
GPSDataGraphic.hideAxis('bottom')
GPSDataGraphic.hideAxis('left')
texttoGPS = pg.TextItem(f"null", anchor=(0.5, 0.5), color='w')
texttoGPS.setFont(font2)
GPSDataGraphic.addItem(texttoGPS)

AltitudeGraphic = win.addPlot(title="Altitude")
AltitudeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
AltitudeGraphic.hideAxis('bottom')
AltitudeGraphic.hideAxis('left')
texttoAltitude = pg.TextItem(f"0.01m", anchor=(0.5, 0.5), color='w')
texttoAltitude.setFont(font2)
AltitudeGraphic.addItem(texttoAltitude)

win.nextCol()

def update_altitude(altitude):
    global texttoAltitude

    AltitudeGraphic.removeItem(texttoAltitude)
    texttoAltitude = pg.TextItem(f"{str(round(float(altitude[0]),2))} m", anchor=(0.5, 0.5), color='w')
    texttoAltitude.setFont(font2)
    AltitudeGraphic.addItem(texttoAltitude)

serial_data = [1] * 19
count = 0

if USE_TCP:
    AVA_model = AVA(tcp_socket.targetIP, CURRENT_STATE, SYSTEM_STATE_POOL)

def update_time():
    global texttoTime

    TimeGraphic.removeItem(texttoTime)
    now = datetime.now()
    texttoTime = pg.TextItem(
        f"{now.strftime('%H:%M:%S')}", anchor=(0.5, 0.5), color="w")
    texttoTime.setFont(font1)
    TimeGraphic.addItem(texttoTime)

def update_gps():
    pass

''' State Switch Case Functions'''
def ABORT():
    return "ABORT"

def IDLE():
    return "IDLE"

def ARMED():
    return "LOGGING"

def ORIGIN():
    return "HOME SET"

def TAKEOFF():
    return "TAKE-OFF"

def update_state():
    global CURRENT_STATE, texttoState

    STATE = int(CURRENT_STATE)
    options = {
        0: ABORT,
        1: IDLE,
        2: ARMED,
        3: ORIGIN,
        4: TAKEOFF,
    }
    state_string = options[STATE]()
    StateGraphic.removeItem(texttoState)
    texttoState = pg.TextItem(str(state_string), anchor=(0.5, 0.5), color='w')
    texttoState.setFont(font2)
    StateGraphic.addItem(texttoState)

def update_battery():
    pass

def tcp_handleshack(commandType: str, commandData: str): 
    ''' Takes the command type and command data and returns raw data'''
    global CURRENT_STATE
    
    tcp_socket.send_data(commandType, commandData)
    raw_data = tcp_socket.receive_data()

    return raw_data

def update_system_state(state):
    '''update system state and return tcp sensor data'''
    return tcp_handleshack("STATE", state)

def write_json(data, filename="web/sensor_data.json"):
    with open(filename, "w") as f:
        json.dump(data, f, indent=3)

def write_data_to_JSON(jcontent):

    with open("web/sensor_data.json") as json_file:
        data = json.load(json_file)
        temp = data["packet"]
        temp[4]["data"]["Pressure"] = float(jcontent[13])
        temp[4]["data"]["Altitude"] = float(jcontent[14])
        temp[4]["data"]["Temperature"] = float(jcontent[15])
        temp[5]["data"]["Pitch"] = float(jcontent[16])
        temp[5]["data"]["Roll"] = float(jcontent[17])
        temp[5]["data"]["Yaw"] = float(jcontent[18])

    write_json(data)
    # print(jcontent)

def update():
    """Update all the data"""
    global serial_interface, serial_data, count, comms, now, CURRENT_STATE, flag

    start = time.perf_counter()

    if USE_TCP:
        serial_data =tcp_handleshack("STATE", CURRENT_STATE)
        serial_data = AVA_model.update(serial_data,CURRENT_STATE)
        
    update_state()
    
    if CURRENT_STATE in (IDLE_state, LOGGING_STATE, ORIGIN_STATE, TAKE_OFF_state):
        update_row_accel(serial_data[0:3])
        update_row_gyro(serial_data[3:6])
        update_row_mag(serial_data[6:9])
        update_pressure(serial_data[13:14])
        update_altitude(serial_data[14:15])
        update_row_temp(serial_data[15:16])
        update_time()
        update_gps()
        update_battery()

        write_data_to_JSON(serial_data)
    else:
        update_time()
        update_gps()
        update_battery()


    finish = time.perf_counter()
    # print(f"[GRAPH] Update Rate: {round(time.perf_counter()-start,5)} seconds ")

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(25)


if __name__ == "__main__":

    # Start Qt event loop unless running in interactive mode
    if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
        QtGui.QApplication.instance().exec_()