import sys
from PyQt4 import QtGui
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d

globalcount = 0
X_Vector = 0
Y_Vector = 0
Rot_Vector = 0

def updatecount():
    global globalcount
    globalcount= globalcount + 1

class GUI(QtGui.QDialog):
    def __init__(self, parent=None):
        count = 0
        super(GUI, self).__init__(parent)

        self.count = 0

        # a figure instance to plot on
        self.figure = plt.figure()

        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)


        # Create Buttons
        self.PlotButton = QtGui.QPushButton('Plot')
        self.PlotButton.setFixedSize(50,50)
        self.PlotButton.clicked.connect(self.plot)
        self.UpButton = QtGui.QPushButton('UP')
        self.UpButton.setFixedSize(50,50)
        self.UpButton.clicked.connect(self.UP)
        self.DownButton = QtGui.QPushButton('DOWN')
        self.DownButton.setFixedSize(50,50)
        self.DownButton.clicked.connect(self.DOWN)
        self.LeftButton = QtGui.QPushButton('LEFT')
        self.LeftButton.setFixedSize(50,50)
        self.LeftButton.clicked.connect(self.LEFT)
        self.RightButton = QtGui.QPushButton('RIGHT')
        self.RightButton.setFixedSize(50,50)
        self.RightButton.clicked.connect(self.RIGHT)
        self.CWButton = QtGui.QPushButton('CW')
        self.CWButton.setFixedSize(50,50)
        self.CWButton.clicked.connect(self.CW)
        self.CCWButton = QtGui.QPushButton('CCW')
        self.CCWButton.setFixedSize(50,50)
        self.CCWButton.clicked.connect(self.CCW)
        self.D11 = QtGui.QLabel('X Vector:')
        self.D11.setFixedWidth(75)
        self.D12 = QtGui.QLabel('0')
        self.D12.setFixedWidth(75)
        self.D21 = QtGui.QLabel('Y Vector:')
        self.D21.setFixedWidth(75)
        self.D22 = QtGui.QLabel('0')
        self.D22.setFixedWidth(75)
        self.D31 = QtGui.QLabel('Rotation:')
        self.D31.setFixedWidth(75)
        self.D32 = QtGui.QLabel('0')
        self.D32.setFixedWidth(75)


        # set the layout
        GL = QtGui.QGridLayout()
        GL.addWidget(self.UpButton,1,2)
        GL.addWidget(self.DownButton,3,2)
        GL.addWidget(self.LeftButton,2,1)
        GL.addWidget(self.RightButton,2,3)
        GL.addWidget(self.CWButton,5,3)
        GL.addWidget(self.CCWButton,5,1)
        GL.addWidget(self.PlotButton,2,2)

        GL2 = QtGui.QGridLayout()
        GL2.addWidget(self.D11,1,1)
        GL2.addWidget(self.D12,1,2)
        GL2.addWidget(self.D21,2,1)
        GL2.addWidget(self.D22,2,2)
        GL2.addWidget(self.D31,3,1)
        GL2.addWidget(self.D32,3,2)

        VL = QtGui.QVBoxLayout()
        VL.addStretch()
        VL.addLayout(GL)
        VL.addLayout(GL2)
        VL.addStretch()

        HL = QtGui.QHBoxLayout()
        HL.addWidget(self.canvas)
        HL.addLayout(VL)
        self.setLayout(HL)
        self.showMaximized()

    def plot(self):

        updatecount()

        LegX = [0, X_Vector]
        LegY = [0, Y_Vector]
        LegZ = [0, Rot_Vector]

        # create an axis
        ax = self.figure.add_subplot(111, projection = '3d')


        # discards the old graph
        ax.hold(False)

        # plot data
        ax.plot(LegX, LegY, LegZ, color='#00FF00')
        ax.set_xlabel('X')

        # refresh canvas
        self.canvas.draw()

    def UP(self):
        global Y_Vector
        Y_Vector=Y_Vector+1
        self.D22.setNum(Y_Vector)
        return

    def DOWN(self):
        global Y_Vector
        Y_Vector=Y_Vector-1
        self.D22.setNum(Y_Vector)
        return

    def LEFT(self):
        global X_Vector
        X_Vector=X_Vector-1
        self.D12.setNum(X_Vector)
        return

    def RIGHT(self):
        global X_Vector
        X_Vector = X_Vector+1
        self.D12.setNum(X_Vector)
        return

    def CW(self):
        global Rot_Vector
        Rot_Vector +=1
        self.D32.setNum(Rot_Vector)
        return

    def CCW(self):
        global Rot_Vector
        Rot_Vector -= 1
        self.D32.setNum(Rot_Vector)



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    main = GUI()
    main.show()

    sys.exit(app.exec_())