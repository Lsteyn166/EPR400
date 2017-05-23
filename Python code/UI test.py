import sys
from PyQt4 import QtGui
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d

globalcount = 0

def updatecount():
    global globalcount
    globalcount= globalcount + 1

class Window(QtGui.QDialog):
    def __init__(self, parent=None):
        count = 0
        super(Window, self).__init__(parent)

        self.count = 0

        # a figure instance to plot on
        self.figure = plt.figure()

        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)


        # Just some button connected to `plot` method
        self.button = QtGui.QPushButton('Plot')
        self.button.clicked.connect(self.plot)

        # set the layout
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.button)
        self.setLayout(layout)
        #self.showMaximized()

    def plot(self):

        updatecount()

        LegX = [0, globalcount]
        LegY = [0, globalcount]
        LegZ = [0, globalcount]

        # create an axis
        ax = self.figure.add_subplot(111, projection = '3d')


        # discards the old graph
        ax.hold(False)

        # plot data
        ax.plot(LegX, LegY, LegZ, color='#00FF00')
        ax.set_xlabel('X')

        # refresh canvas
        self.canvas.draw()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    main = Window()
    main.show()

    sys.exit(app.exec_())