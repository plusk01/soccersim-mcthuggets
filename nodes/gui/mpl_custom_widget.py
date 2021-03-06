#!/usr/bin/env python
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4 import NavigationToolbar2QT as NavigationToolbar
from matplotlib.backend_bases import NavigationToolbar2

from matplotlib.figure import Figure

class MyMplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=10, height=12, dpi=100, sharex=None, sharey=None):
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor='#CDCDCD')
        self.ax = self.fig.add_subplot(111, sharex=sharex, sharey=sharey)
        self.fig.subplots_adjust(left=0.1, bottom=0.15, right=0.9, top=0.9)
        self.xtitle="x-Axis"
        self.ytitle="y-Axis"
        self.PlotTitle = "Subplot"
        self.grid_status = True
        self.xaxis_style = 'linear'
        self.yaxis_style = 'linear'
        self.format_labels()
        self.ax.hold(True)
        FigureCanvas.__init__(self, self.fig)
        #self.fc = FigureCanvas(self.fig)
        FigureCanvas.setSizePolicy(self,
            QSizePolicy.Expanding,
            QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def format_labels(self):
        if isinstance(self.ax, dict):
            axes = self.ax.itervalues()
        else:
            axes = [self.ax]

        for ax in axes:
            ax.set_title(self.PlotTitle)
            ax.title.set_fontsize(10)
            ax.set_xlabel(self.xtitle, fontsize=9)
            ax.set_ylabel(self.ytitle, fontsize=9)
            labels_x = ax.get_xticklabels()
            labels_y = ax.get_yticklabels()

            for xlabel in labels_x:
                xlabel.set_fontsize(8)
            for ylabel in labels_y:
                ylabel.set_fontsize(8)

    def sizeHint(self):
        w, h = self.get_width_height()
        return QSize(w, h)

    def minimumSizeHint(self):
        return QSize(10, 10)

    def sizeHint(self):
        w, h = self.get_width_height()
        return QSize(w, h)

    def minimumSizeHint(self):
        return QSize(10, 10)


class MyNavigationToolbar(NavigationToolbar) :
    def __init__(self, parent, canvas, direction='h') :
        #NavigationToolbar.__init__(self, parent, canvas)
        #self.layout = QVBoxLayout( self )

        self.canvas = canvas
        # QWidget.__init__( self, parent )

        # if direction == 'h':
        #     self.layout = QHBoxLayout( self )
        # else:
        #     self.layout = QVBoxLayout( self )

        # self.layout.setMargin( 2 )
        # self.layout.setSpacing( 0 )

        super(MyNavigationToolbar, self).__init__(parent, canvas, direction)

class MPL_Widget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.canvas = MyMplCanvas()
        self.toolbar = MyNavigationToolbar(self.canvas, self.canvas, direction='v')
        self.toolbar.hide()
        self.hbox = QHBoxLayout()
        # self.hbox.addWidget(self.toolbar)
        self.hbox.addWidget(self.canvas)
        self.setLayout(self.hbox)

