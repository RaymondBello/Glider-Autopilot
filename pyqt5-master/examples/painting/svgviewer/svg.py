from PyQt5.QtCore import QFile, QSize, Qt, QPointF
from PyQt5.QtGui import QBrush, QColor, QImage, QPainter, QPixmap, QPen, QTransform
from PyQt5.QtWidgets import (QActionGroup, QApplication, QFileDialog,
                             QGraphicsItem, QGraphicsRectItem, QGraphicsScene, QGraphicsView,
                             QMainWindow, QMenu, QMessageBox, QWidget)
from PyQt5.QtOpenGL import QGL, QGLFormat, QGLWidget
from PyQt5.QtSvg import QGraphicsSvgItem


class SvgView(QGraphicsView):
    Native, OpenGL, Image = range(3)

    def __init__(self, parent=None):
        super(SvgView, self).__init__(parent)
        self.renderer = SvgView.Native
        self.svgItem = None
        self.backgroundItem = None
        self.outlineItem = None
        self.image = QImage()

        self.setScene(QGraphicsScene(self))
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        # Prepare background check-board pattern.
        tilePixmap = QPixmap(64, 64)
        tilePixmap.fill(Qt.white)
        tilePainter = QPainter(tilePixmap)
        color = QColor(220, 220, 220)
        tilePainter.fillRect(0, 0, 32, 32, color)
        tilePainter.fillRect(32, 32, 32, 32, color)
        tilePainter.end()

        self.setBackgroundBrush(QBrush(tilePixmap))

    def openFile(self, svg_file):
        if not svg_file.exists():
            return

        s = self.scene()

        self.resetTransform()

        self.svgItem = QGraphicsSvgItem(svg_file.fileName())
        # self.svgItem.setFlags(QGraphicsItem.ItemIsSelectable)
        # self.svgItem.setFlags(QGraphicsItem.ItemIsMovable)
        # self.svgItem.setCacheMode(QGraphicsItem.NoCache)
        self.svgItem.setZValue(1)

        self.svgItem2 = QGraphicsSvgItem(
            QFile("files/horizon_back.svg").fileName())
        # self.svgItem2.setFlags(QGraphicsItem.ItemIsSelectable)
        # self.svgItem2.setFlags(QGraphicsItem.ItemIsMovable)
        # self.svgItem2.setFlags(QGraphicsItem.ItemClipsToShape)
        # self.svgItem2.setCacheMode(QGraphicsItem.NoCache)
        self.svgItem2.setZValue(2)

        self.svgItem3 = QGraphicsSvgItem(
            QFile("files/horizon_ball.svg").fileName())
        # self.svgItem3.setFlags(QGraphicsItem.ItemIsSelectable)
        # self.svgItem3.setFlags(QGraphicsItem.ItemIsMovable)
        # self.svgItem3.setFlags(QGraphicsItem.ItemClipsToShape)
        # self.svgItem3.setCacheMode(QGraphicsItem.NoCache)
        self.svgItem3.setZValue(3)

        self.svgItem4 = QGraphicsSvgItem(
            QFile("files/horizon_circle.svg").fileName())
        # self.svgItem4.setFlags(QGraphicsItem.ItemIsSelectable)
        # self.svgItem4.setFlags(QGraphicsItem.ItemIsMovable)
        # self.svgItem4.setFlags(QGraphicsItem.ItemClipsToShape)
        # self.svgItem4.setCacheMode(QGraphicsItem.NoCache)
        self.svgItem4.setZValue(4)

        self.svgItem5 = QGraphicsSvgItem(
            QFile("files/horizon_mechanics.svg").fileName())
        # self.svgItem5.setFlags(QGraphicsItem.ItemIsSelectable)
        # self.svgItem5.setFlags(QGraphicsItem.ItemIsMovable)
        # self.svgItem5.setFlags(QGraphicsItem.ItemClipsToShape)
        # self.svgItem5.setCacheMode(QGraphicsItem.NoCache)
        self.svgItem5.setZValue(5)

        self.svgItem6 = QGraphicsSvgItem(
            QFile("files/fi_circle.svg").fileName())
        # self.svgItem6.setFlags(QGraphicsItem.ItemIsSelectable)
        # self.svgItem6.setFlags(QGraphicsItem.ItemIsMovable)
        # self.svgItem6.setFlags(QGraphicsItem.ItemClipsToShape)
        # self.svgItem6.setCacheMode(QGraphicsItem.NoCache)
        self.svgItem6.setZValue(6)

        s.addItem(self.svgItem)
        s.addItem(self.svgItem2)
        s.addItem(self.svgItem3)
        s.addItem(self.svgItem4)
        s.addItem(self.svgItem5)
        s.addItem(self.svgItem6)

        
        # self.svgItem3.setPos(QPointF(0,0))
        print(self.svgItem3.scenePos().x())
        
        # self.svgItem3.setY((27.5*(0))/10)

        
        gr = s.createItemGroup([self.svgItem2, self.svgItem3, self.svgItem4, self.svgItem6])
        offset = gr.sceneBoundingRect().center()
        tranf = QTransform();
        tranf.translate(offset.x(), offset.y())
        tranf.rotate(0)
        tranf.translate(-offset.x(), -offset.y())
        gr.setTransform(tranf)
        
        s.destroyItemGroup(gr)
        s.update()
        
        print(self.svgItem3.scenePos().x())
        self.svgItem3.setPos(QPointF(self.svgItem3.scenePos().x()-((27.5*(0))/10),self.svgItem3.scenePos().y()+((27.5*(0))/10)))
        
        

        print(f"{self.svgItem3.boundingRect()}")

        # self.svgItem3.setScale(1)                           # Scale Command


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.currentPath = ''

        self.view = SvgView()
        self.setCentralWidget(self.view)
        self.setWindowTitle("SVG Viewer")

    def openFile(self, path=None):
        if not path:
            path, _ = QFileDialog.getOpenFileName(self, "Open SVG File",
                                                  self.currentPath, "SVG files (*.svg *.svgz *.svg.gz)")

        if path:
            svg_file = QFile(path)
            if not svg_file.exists():
                QMessageBox.critical(self, "Open SVG File",
                                     "Could not open file '%s'." % path)

                return

            self.view.openFile(svg_file)

            if not path.startswith(':/'):
                self.currentPath = path
                self.setWindowTitle("%s - SVGViewer" % self.currentPath)

            self.resize(self.view.sizeHint() +
                        QSize(80, 80 + self.menuBar().height()))


if __name__ == '__main__':

    import sys

    app = QApplication(sys.argv)

    window = MainWindow()
    if len(sys.argv) == 2:
        window.openFile(sys.argv[1])
    else:
        window.openFile('files/fi_box.svg')
    window.show()
    sys.exit(app.exec_())
