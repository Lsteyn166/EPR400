#################################################################
#                                                               #
# Author: L Steyn                                               #
#                                                               #
# Code from Inverse Kinematics merged with UI test code         #
#                                                               #
#################################################################

import sys
from PyQt4 import QtGui
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
import math
from matplotlib.patches import Circle
import matplotlib
# Predefine Vectors
X_Vector = 0
Y_Vector = 0
Rot_Vector = 0


# Predefine variables
Z_body = 0.5                                            # Height of the robot body
A = 0.5                                                 # Leg segment A length
B = 1                                                   # Leg segment B length
C = 1                                                   # Leg segment C length
Radius = 1                                              # Radius of robot body
Z = 0
Time = 0
destination = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
current = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
reset =[False,False,False,False,False]

# Create GUI class with all the components necessary to create and maintain the GUI
class GUI(QtGui.QDialog):

    def __init__(self, parent=None):                    # Main function of the GUI class. Inits everything
        super(GUI, self).__init__(parent)
        self.figure = plt.figure()                      # Create instance of matplotlib figure for GUI
        GUI.canvas = FigureCanvas(self.figure)         # Put this figure on a canvas object
        plt.ion()

        font = {'family': 'Serif',
                'weight': 'normal',
                'size': 20}

        matplotlib.rc('font', **font)

        self.setStyleSheet("QWidget {font: 20pt Times}")


        # Create UI widgets
        self.PlotButton = QtGui.QPushButton('Step')     # Plot button
        self.PlotButton.setFixedSize(90, 90)
        self.PlotButton.clicked.connect(self.Step)
        self.UpButton = QtGui.QPushButton('UP')         # Forward button
        self.UpButton.setFixedSize(90, 90)
        self.UpButton.clicked.connect(self.UP)
        self.DownButton = QtGui.QPushButton('DOWN')     # Reverse button
        self.DownButton.setFixedSize(90, 90)
        self.DownButton.clicked.connect(self.DOWN)
        self.LeftButton = QtGui.QPushButton('LEFT')     # Left button
        self.LeftButton.setFixedSize(90, 90)
        self.LeftButton.clicked.connect(self.LEFT)
        self.RightButton = QtGui.QPushButton('RIGHT')   # Right button
        self.RightButton.setFixedSize(90, 90)
        self.RightButton.clicked.connect(self.RIGHT)
        self.CWButton = QtGui.QPushButton('CW')         # Rotate clockwise button
        self.CWButton.setFixedSize(90, 90)
        self.CWButton.clicked.connect(self.CW)
        self.CCWButton = QtGui.QPushButton('CCW')       # Rotate counterclockwise button
        self.CCWButton.setFixedSize(90, 90)
        self.CCWButton.clicked.connect(self.CCW)
        self.D11 = QtGui.QLabel('X Vector:')            # X vector text label
        self.D11.setFixedWidth(135)
        self.D12 = QtGui.QLabel('0')                    # X vector numerical label
        self.D12.setFixedWidth(135)
        self.D21 = QtGui.QLabel('Y Vector:')            # Y vector text label
        self.D21.setFixedWidth(135)
        self.D22 = QtGui.QLabel('0')                    # Y vector numerical label
        self.D22.setFixedWidth(135)
        self.D31 = QtGui.QLabel('Rotation:')            # Rotation text label
        self.D31.setFixedWidth(135)
        self.D32 = QtGui.QLabel('0')                    # Rotation numerical label
        self.D32.setFixedWidth(135)
        self.AutoPlot = QtGui.QCheckBox('Auto Plot')    # Checkbox to enable plotting on press of a a directional button

        # Set all widgets in layouts
        GL = QtGui.QGridLayout()                        # Grid layout for buttons
        GL.addWidget(self.UpButton, 1, 2)
        GL.addWidget(self.DownButton, 3, 2)
        GL.addWidget(self.LeftButton, 2, 1)
        GL.addWidget(self.RightButton, 2, 3)
        GL.addWidget(self.CWButton, 5, 3)
        GL.addWidget(self.CCWButton, 5, 1)
        GL.addWidget(self.PlotButton, 2, 2)

        GL2 = QtGui.QGridLayout()                       # Grid layout for labels
        GL2.addWidget(self.D11, 1, 1)
        GL2.addWidget(self.D12, 1, 2)
        GL2.addWidget(self.D21, 2, 1)
        GL2.addWidget(self.D22, 2, 2)
        GL2.addWidget(self.D31, 3, 1)
        GL2.addWidget(self.D32, 3, 2)

        VL = QtGui.QVBoxLayout()                        # Vertical layout for grid layouts
        VL.addStretch()
        VL.addLayout(GL)
        VL.addWidget(self.AutoPlot)
        VL.addLayout(GL2)
        VL.addStretch()

        HL = QtGui.QHBoxLayout()                        # Horizontal layout for canvas and vertical layout
        HL.addWidget(self.canvas)
        HL.addLayout(VL)

        self.setLayout(HL)
        self.showMaximized()
        self.Step()

    def Step(self):                                     # Function for updating the plot in the UI
        GUI.ax = self.figure.add_subplot(111, projection='3d')  # Create axis on plot
        self.figure.tight_layout()
        GUI.ax.hold(True)
        ConfigurePlot()
        for leg in range(1, 6):
            # Determine desired coordinates of a given leg from x,y vectors and rotation
            X, Y = Vector(-X_Vector, -Y_Vector, leg, -Rot_Vector)
            current[2*leg-2] = X
            current[2 * leg - 1] = Y
            reset[leg - 1] = CheckBound(current[2*leg-2], current[2 * leg - 1], leg)

        for leg in range(1, 6):
            #Reset the legs that need it
            if reset[leg-1] == True:
                ResetLeg(leg)
            X = current[2*leg-2]
            Y = current[2*leg-1]
            # Determine Servo angles for desired position
            Theta, Phi, Alpha = IK(X, Y, Z, leg)
            # Plot result
            Plotleg(Theta, Phi, Alpha, leg, False)
            # Find position of little orange markers
            x, y = RotateLeg(2.5, 0, leg)
            # Plot little orange markers for home position
            self.ax.plot([x, x], [y, y], [0, -0.05], color='#FFAF00',linewidth=4)

        GUI.canvas.draw()                              # Refresh canvas

    def UP(self):                                       # Callback function for Button
        global Y_Vector
        Y_Vector += .1
        if Y_Vector > 0.5:
            Y_Vector = 0.5
        self.D22.setNum(Y_Vector)
        if self.AutoPlot.isChecked():
            self.Step()
        return

    def DOWN(self):                                     # Callback function for Button
        global Y_Vector
        Y_Vector -= .1
        if Y_Vector < -0.5:
            Y_Vector = -0.5
        self.D22.setNum(Y_Vector)
        if self.AutoPlot.isChecked():
            self.Step()
        return

    def LEFT(self):                                     # Callback function for Button
        global X_Vector
        X_Vector -= .1
        if X_Vector < -0.5:
            X_Vector = -0.5
        self.D12.setNum(X_Vector)
        if self.AutoPlot.isChecked():
            self.Step()
        return

    def RIGHT(self):                                    # Callback function for Button
        global X_Vector
        X_Vector += .1
        if X_Vector > 0.5:
            X_Vector = 0.5
        self.D12.setNum(X_Vector)
        if self.AutoPlot.isChecked():
            self.Step()
        return

    def CW(self):                                       # Callback function for Button
        global Rot_Vector
        Rot_Vector += 1
        self.D32.setNum(Rot_Vector)
        if self.AutoPlot.isChecked():
            self.Step()
        return

    def CCW(self):                                      # Callback function for Button
        global Rot_Vector
        Rot_Vector -= 1
        self.D32.setNum(Rot_Vector)
        if self.AutoPlot.isChecked():
            self.Step()
        return
# End of class GUI

# Functions from inverse kinematics calculations

def Plotleg(Theta, Phi, Alpha, leg, Dim):
    ""
    "This function plots a leg after the inverse kinematics are calculated"
    # Call the translation function
    offsetx, offsety = TranslateLeg(leg)

    # Plot segment A
    if Dim == True:
        plotColour = '#FFAFAF'
    else:
        plotColour = '#FF0000'
    LegX = [offsetx, offsetx + A * math.sin(Theta)]
    LegY = [offsety, offsety + A * math.cos(Theta)]
    LegZ = [Z_body, Z_body]
    GUI.ax.plot(LegX, LegY, LegZ, plotColour,linewidth=2)

    # Plot segment B
    if Dim == True:
        plotColour = '#AFFFAF'
    else:
        plotColour = '#00FF00'
    LegX = [offsetx + A * math.sin(Theta), offsetx + A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta)]
    LegY = [offsety + A * math.cos(Theta), offsety + A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta)]
    LegZ = [Z_body, Z_body + B * math.cos(Phi)]
    GUI.ax.plot(LegX, LegY, LegZ, plotColour,linewidth=2)

    # Plot segment C
    if Dim == True:
        plotColour = '#AFAFFF'
    else:
        plotColour = '#0000FF'
    LegX = [offsetx + A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta),
            offsetx + A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta) + C * math.sin(
                Phi + Alpha - math.radians(90)) * math.sin(Theta)]
    LegY = [offsety + A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta),
            offsety + A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta) + C * math.sin(
                Phi + Alpha - math.radians(90)) * math.cos(Theta)]
    LegZ = [Z_body + B * math.cos(Phi), Z_body + B * math.cos(Phi) + C * math.cos(Phi + Alpha - math.radians(90))]
    GUI.ax.plot(LegX, LegY, LegZ, plotColour,linewidth=2)
    # print (LegX[1], LegY[1], LegZ[1]), "\n\r"
    return

def IK(X, Y, Z, leg):
    ""
    "This function calculates the servo positions required for specific coordinates"
    # Call the translatiom function
    Theta = math.atan2(X, Y)  # Calculate Theta
    # print "Theta", leg, "=", math.degrees(Theta)
    ###Calculate Phi & Alpha:
    x1 = A * math.sin(Theta)
    y1 = A * math.cos(Theta)
    z1 = Z_body
    Reach = math.sqrt((X - x1) ** 2 + (Y - y1) ** 2)  # Horizontal Distance from Joint 2 to foot
    TwoToFoot = math.sqrt((X - x1) ** 2 + (Y - y1) ** 2 + (Z - z1) ** 2)  # Absolute Distance from Joint 1 to foot
    # print("TwoToFoot")
    # print(TwoToFoot)
    x = (B ** 2 + C ** 2 - TwoToFoot ** 2) / (2 * B * C)
    # print(x)
    d = math.acos(x)
    # print("d")
    # print(math.degrees(d))
    Alpha = 270 - math.degrees(d)
    Alpha = math.radians(Alpha)
    # print "Alpha", leg, "=", math.degrees(Alpha)
    c = math.asin(C * math.sin(d) / TwoToFoot)
    # print("c")
    # print(math.degrees(c))
    e = math.atan2(Reach, Z_body - Z)
    # print("e")
    # print(math.degrees(e))
    Phi = 180 - math.degrees(c) - math.degrees(e)
    Phi = math.radians(Phi)
    # print "Phi  ", leg, "=", math.degrees(Phi)
    return Theta, Phi, Alpha

def Rotate(xi, yi, angle):
    ""
    "This function rotates the coordinates of a specific point around the origin by a certain angle(degrees)"
    Angle = math.radians(angle)
    xo = xi * math.cos(Angle) - yi * math.sin(Angle)
    yo = xi * math.sin(Angle) + yi * math.cos(Angle)
    return xo, yo

def RotateLeg(x, y, leg):
    ""
    "This function rotates the coordinates of a specific leg to its required position"
    Angle = math.radians((leg - 1) * 72)
    X = x * math.cos(Angle) - y * math.sin(Angle)
    Y = x * math.sin(Angle) + y * math.cos(Angle)
    return X, Y

def TranslateLeg(leg):
    ""
    "This function is called from the plotleg function to set the necessary offsets for plotting."
    "It calculates the offset distance from shoulder joint to robot centre"
    x = Radius * math.cos(math.radians(72 * (leg - 1)))
    y = Radius * math.sin(math.radians(72 * (leg - 1)))
    return x, y

def Vector(x, y, leg, Rot):
    ""
    "This function determines the required position of the foot given a specific vector and rotation"
    # First do the translation vector
    Angle = math.atan2(y, x)
    Magnitude = math.sqrt(x ** 2 + y ** 2)  # Calculate actual magnitude
    if (Magnitude > 0.5):  # Limit magnitude to 0.5
        Magnitude = 0.5
    NeutralX, NeutralY = RotateLeg(2.5, 0, leg)
    a, b = RotateLeg(Radius, 0, leg)
    NeutralX = NeutralX - a
    NeutralY = NeutralY - b
    X = NeutralX + Magnitude * math.cos(Angle)
    Y = NeutralY + Magnitude * math.sin(Angle)

    # Now the rotation part
    offsetx, offsety = TranslateLeg(leg)  # Move system origin to robot center
    X = X + offsetx
    Y = Y + offsety
    X, Y = Rotate(X, Y, -10 * Rot)  # Rotate around origin
    X = X - offsetx
    Y = Y - offsety


    destination[2*leg-2] = X
    destination[2*leg-1] = Y
    return X, Y

def CheckBound(x, y, leg):
    ""
    "This function checks if a leg needs to be reset by determining whether it is within range from its neutral position"
    NeutralX, NeutralY = RotateLeg(1.5, 0, leg)
    deltaX = NeutralX - x
    deltaY = NeutralY - y
    absDist = math.sqrt(deltaX**2+deltaY**2)
    if absDist > 0.51:
        return True
    else:
        return False

def ResetLeg(leg):
    ""
    "This function resets a leg to the postion furthest in the direction of robot movement"
    #Pick up the leg
    Z = 0.5
    X = destination[2*leg-2]
    Y = destination[2*leg-1]
    # Determine Servo angles for desired position
    Theta, Phi, Alpha = IK(X, Y, Z, leg)
    # Plot result
    Plotleg(Theta, Phi, Alpha, leg, True)
    # plt.pause(0.5)
    # GUI.canvas.draw()
    #Move to reset position
    X, Y = Vector(X_Vector, Y_Vector, leg, Rot_Vector)
    current[2*leg-2] = X
    current[2*leg-1] = Y
    Theta, Phi, Alpha = IK(X, Y, Z, leg)
    Plotleg(Theta, Phi, Alpha, leg, True)
    # GUI.canvas.draw
    #Put leg down
    Z = 0
    Theta, Phi, Alpha = IK(X, Y, Z, leg)
    Plotleg(Theta, Phi, Alpha, leg, False)
    return

def ConfigurePlot():
    ""
    "This function creates a 3D plot and configures axes and labels"
    blank = [0, 0]
    GUI.ax.plot(blank, blank, blank, label="Segment A", color='#FF0000' )
    GUI.ax.plot(blank, blank, blank, label="Segment B", color='#00FF00')
    GUI.ax.plot(blank, blank, blank, label="Segment C", color='#0000FF')
    GUI.ax.plot(blank, blank, blank, label="Chassis", color='#AF00FF')
    GUI.ax.plot(blank, blank, blank, label="Feet neutral", color='#FFAF00')

    p = Circle((0, 0), Radius, color='#AF00FF')
    GUI.ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p, z=Z_body, zdir="z")

    GUI.ax.legend(loc = 2)
    GUI.ax.set_xlabel('X')
    GUI.ax.set_ylabel('Y')
    GUI.ax.set_zlabel('Z')
    GUI.ax.axis([-2.5, 2.5, -2.5, 2.5])
    GUI.ax.set_zlim3d(0, 5)

if __name__ == '__main__':                              # Start of main application
    app = QtGui.QApplication(sys.argv)
    main = GUI()                                        # Start a GUI instance
    main.show()

    sys.exit(app.exec_())                               # Stop when GUI is closed