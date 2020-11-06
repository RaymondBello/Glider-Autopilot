import sys
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from pyqtlet import L, MapWidget
import random
import time

class MapWindow(QWidget):
    def __init__(self):
        # Setting up the widgets and layout
        super().__init__()
        self.mapWidget = MapWidget()
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.mapWidget)
        self.setLayout(self.layout)

        # Working with the maps with pyqtlet
        self.map = L.map(self.mapWidget)
        self.map.setView([45.404290, -75.706760], 10)
        L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}{r}.png').addTo(self.map)
        self.marker = L.marker([45.404179, -75.706679])
        self.marker.bindPopup('Maps are a treasure.')
        self.map.addLayer(self.marker)
        self.show()
    
    def update(self):
        for i in range(10):
            self.marker = L.marker([45.404179-(i*0.01),-75.706679+(i*0.01)])
            self.marker.bindPopup('Maps are a treasure.')
            self.map.addLayer(self.marker)
            self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = MapWindow()
    widget.update()
    idx = 0
    sys.exit(app.exec_())

    


    
