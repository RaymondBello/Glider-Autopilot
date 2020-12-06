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


class GCS_Plotter:
    '''This is the base class for the Ground Control Station Plotter'''
    use_TCP = False

    def __init__(self):
        pass

    @classmethod
    def from_flags(cls):
        cls.use_TCP = True if (sys.argv[-1].lower() in ["true", "t"]) else False
        return cls()