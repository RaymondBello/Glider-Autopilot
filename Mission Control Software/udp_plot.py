from datetime import datetime
from communication import Comms
from TCP import TCP_Manager
import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import QPushButton
import numpy as np
import time
import sys

USE_TCP = False

# Serial connection is setup here
# comms = Comms()

# TCP connection is setup here
if USE_TCP:
    print("[SET-UP] : Setting up TCP connection")
    try:
        tcp_socket = TCP_Manager()
        print("[SET-UP] : TCP Connection Establied!")
    except expression as identifier:
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
# RETURN_TO_HOME = 5


# GUI WINDOW SIZE CONSTANTS
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700

data1 = [0] * 200
data2 = [0] * 200
data3 = [0] * 200

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client.bind(("192.168.0.12", 64886))

# while True:
#     packet, addr = client.recvfrom(100)
#     clean_packet = str(packet).split("'")[1].split(",")
#     print(clean_packet)


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

topSection = win.addLayout(colspan=10)


def abort_state():
    global CURRENT_STATE
    CURRENT_STATE = int(not bool(CURRENT_STATE))
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

win.nextRow()

proxy5 = QtGui.QGraphicsProxyWidget()
button_return_to_home = QtGui.QPushButton('RETURN')
button_return_to_home.setStyleSheet(ButtonStyle_yellow)
button_return_to_home.clicked.connect(return_home_state)
proxy5.setWidget(button_return_to_home)
topSection.addItem(proxy5)

topSection.nextCol()


graph1 = win.addPlot(title="Accelerometer X")
graph2 = win.addPlot(title="Accelerometer Y")
graph3 = win.addPlot(title="Accelerometer Z")
# graph1.addLegend(offset=(1, 1))
# graph2.addLegend(offset=(1, 1))
# graph3.addLegend(offset=(1, 1))


# data1 = np.random.normal(size=300)

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


def update_row_1(acc_values):
    """Update graph-> shifts data in the array one sample left, appends new value"""

    global data1, data2, data3, graph1_curve, graph2_curve, graph3_curve, ptr1

    shift_array(data1)
    shift_array(data2)
    shift_array(data3)
    # data2[:-1] = data2[1:]

    if ptr1 < 50:
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
    name="Gyro_X",
    # labels={"bottom": graph5.setLabel("bottom", text="Time", units="s"),},
)

graph6.setRange(xRange=[-200, 0])
graph6.setLimits(xMax=0)
graph6_curve = graph6.plot(
    pen=(0, 0, 150),
    name="Gyro_X",
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
texttoBattery = pg.TextItem(f"11.2V", anchor=(0.5, 0.5), color='w')
texttoBattery.setFont(font2)
BatteryGraphic.addItem(texttoBattery)


def update_row_2(gyro_values):
    """Plot in chunks, adding one new plot curve for every 100 samples"""

    global data4, data5, data6, ptr2

    if ptr2 < 50:
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


# Plot in chunks, adding one new plot curve for every 100 samples
chunkSize = 100

# Remove chunks after we have 10
maxChunks = 10

startTime = pg.ptime.time()

win.nextRow()

graph7 = win.addPlot(colspan=1, title="Internal Temperature")
# graph7.setLabel("bottom", "Time", "s")
graph7.setXRange(-10, 0)

curves = []
data7 = np.empty((chunkSize + 1, 2))
ptr3 = 0


def update_row_3(temp_values):
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

AltitudeGraphic = win.addPlot(title="Altitude ")
AltitudeGraphic.setRange(QtCore.QRectF(-50, -50, 100, 100))
AltitudeGraphic.hideAxis('bottom')
AltitudeGraphic.hideAxis('left')
texttoAltitude = pg.TextItem(f"31m", anchor=(0.5, 0.5), color='w')
texttoAltitude.setFont(font2)
AltitudeGraphic.addItem(texttoAltitude)
# win.nextCol()


def udp_get_packet_data():
    """Receive udp packet, parses data and returns clean_data_packet"""

    global client, addr
    packet, addr = client.recvfrom(100)
    clean_packet = str(packet).split("'")[1].split(",")
    # print(addr)
    print(clean_packet[:3], clean_packet[3:4], clean_packet[4:], addr)

    MSG = "Test String"

    client.sendto(bytes(MSG, "utf-8"), addr)

    # print(round(float(clean_packet[0]), 2))
    return clean_packet


def udp_send_packet_data():
    global client, addr

    MSG = "Test String"

    client.sendto(bytes(MSG, "utf-8"), addr)


serial_data = [1] * 7
count = 0


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

# State Switch Case Functions


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


def update_altitude():
    pass

# update all plots

def tcp_handleshack(commandType: str, commandData: str): 
    ''' Takes the command type and command data and returns raw data'''
    global CURRENT_STATE
    

    tcp_socket.send_data(commandType, commandData)
    raw_data = tcp_socket.receive_data()
    print(raw_data)

    # return raw_data

def update_system_state(state):
    '''update system state and return tcp sensor data'''
    return tcp_handleshack("STATE", state)

    
    

def update():
    """Update all the data"""
    global serial_interface, serial_data, count, comms, now, CURRENT_STATE

    start = time.perf_counter()

    if USE_TCP:
        tcp_handleshack("STATE", CURRENT_STATE)
        pass
    update_state()
    
    if CURRENT_STATE in (IDLE_state, LOGGING_STATE, ORIGIN_STATE, TAKE_OFF_state):
        update_row_1(serial_data[:3])
        update_row_2(serial_data[4:])
        update_row_3(serial_data[3:4])
        update_time()
        update_gps()
        update_battery()
        update_altitude()
    else:
        update_time()
        update_gps()
        update_battery()
        update_altitude()

    finish = time.perf_counter()
    # print(f"[GRAPH] Update Rate: {round(time.perf_counter()-start,5)} seconds ")

    count += 1


timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
# timer.timeout.connect(udp_get_packet_data)
timer.start(10)


if __name__ == "__main__":

    # Start Qt event loop unless running in interactive mode
    if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
        QtGui.QApplication.instance().exec_()
