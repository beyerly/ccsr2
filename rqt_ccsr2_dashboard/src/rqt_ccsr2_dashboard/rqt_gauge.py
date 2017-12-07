

import os
import math 
  
from python_qt_binding.QtWidgets import QWidget, QGraphicsScene
from python_qt_binding.QtGui import QPainter, QPixmap, QColor, QPen, QPolygonF, QBrush, QFont
from python_qt_binding.QtCore import Qt, QPointF, QRectF

class rqt_gauge():

    def __init__(self, vrange, size, unit, type=0):
        self.scene = QGraphicsScene()
        self.size = size
        self.range = vrange
        
        self.bg = QPixmap("src/ccsr2/rqt_ccsr2_dashboard/resource/gauge_bg1.jpg").scaled(self.size, self.size)
        self.bgpainter = QPainter(self.bg);
        
        angle = 0.3
        font = QFont()
        font.setPixelSize(8);
        self.bgpainter.setFont(font);
        self.bgpainter.setPen(QPen(Qt.darkBlue, 1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin));
        self.bgpainter.setBrush(QBrush(Qt.darkBlue)) 
        self.bgpainter.drawText(QRectF(self.size/2-10, self.size/2-10, 20, 5), Qt.AlignCenter, str(unit));
#        self.bgpainter.drawRect(QRectF(50, 50, 10, 10))
        for j in range(0, 10):
            x = (self.size-35)*math.sin(-angle)/2 + (self.size-15)/2
            y = (self.size-35)*math.cos(-angle)/2 + (self.size-5)/2
            self.bgpainter.drawText(QRectF(x, y, 15, 5), Qt.AlignCenter, str(j*(self.range/10)));
#            x = (self.size-20)*math.sin(-angle)/2 + (self.size)/2
#            y = (self.size-20)*math.cos(-angle)/2 + (self.size)/2
#            self.bgpainter.drawEllipse(QPointF(x, y), 2, 2);
            angle = angle + (2*math.pi - 0.3)/10
            
        self.scene.addPixmap(self.bg);





#        self.scene.addPixmap(self.pixmap);
        self.pixmap = QPixmap(self.size, self.size)
        self.pixmap.fill(Qt.transparent) 
        self.qpainter = QPainter(self.pixmap);
        self.qpainter.setCompositionMode(QPainter.CompositionMode_Source)
        self.needle = self.scene.addPixmap(self.pixmap);
        self.x = 0
        self.y = 0

        self.knob = QPixmap(self.size, self.size)
        self.knob.fill(Qt.transparent)
        self.knobPainter = QPainter(self.knob);
        
        self.knobPainter.setPen(QPen(Qt.black, 1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin));
        self.knobPainter.setBrush(QBrush(Qt.black)) 
        self.knobPainter.drawEllipse(QPointF(self.size/2, self.size/2), 4, 4);
        self.scene.addPixmap(self.knob);


        
    def draw_needle(self, value):
        if value:
            color = Qt.red
        else:
            color = Qt.transparent
        self.qpainter.setPen(QPen(color, 1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin));
        self.qpainter.setBrush(QBrush(color)) 
        polygon = QPolygonF() 
        polygon.append(QPointF(self.size/2 -3,self.size/2))
        polygon.append(QPointF(self.size/2 + 3,self.size/2))
        polygon.append(QPointF(self.x,self.y))
        self.qpainter.drawPolygon(polygon);

    def draw_gauge(self, value):
        
        self.draw_needle(False)
        
        angle = (2*math.pi * value/self.range) + 0.3
        self.x = 0.7*self.size*math.sin(-angle)/2 + self.size/2
        self.y = 0.7*self.size*math.cos(-angle)/2 + self.size/2

        self.draw_needle(True)
        self.needle.setPixmap(self.pixmap);
        pass 
