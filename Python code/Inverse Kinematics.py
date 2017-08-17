# Inverse Kinematics code test by plotting in 3D space
# Luan Steyn
# 2017
#import dependencies
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.animation as animation
from matplotlib.patches import Circle, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
#Declare variables used

X=0                       #Result variables
Y=0
Z=0
Z_body=30                  #Input variables
Theta=90
Phi=0
Alpha=0
Reach=0
A=50
B=106
C=130
Radius=90
Leg1MountAngle=0
AbsDistance1=0

####################################  FUNCTIONS  ########################################################

def Plotleg (Theta,Phi,Alpha,leg):
    ""
    "This function plots a leg after the inverse kinematics are calculated"
    # Call the translatiom function
    offsetx, offsety = TranslateLeg(leg)

    # Plot segment A
    LegX = [offsetx, offsetx+A * math.sin(Theta)]
    LegY = [offsety, offsety+A * math.cos(Theta)]
    LegZ = [Z_body, Z_body]
    ax.plot(LegX, LegY, LegZ, color='#FF0000')

    # Plot segment B
    LegX = [offsetx+A * math.sin(Theta),offsetx+ A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta)]
    LegY = [offsety+A * math.cos(Theta),offsety+ A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta)]
    LegZ = [Z_body, Z_body + B * math.cos(Phi)]
    ax.plot(LegX, LegY, LegZ, color='#00FF00')

    # Plot segment C
    LegX = [offsetx+A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta),offsetx+A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta) + C * math.sin(Phi + Alpha - math.radians(90)) * math.sin(Theta)]
    LegY = [offsety+A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta),offsety+A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta) + C * math.sin(Phi + Alpha - math.radians(90)) * math.cos(Theta)]
    LegZ = [Z_body + B * math.cos(Phi), Z_body + B * math.cos(Phi) + C * math.cos(Phi + Alpha - math.radians(90))]
    ax.plot(LegX, LegY, LegZ, color='#0000FF')
    print (LegX[1], LegY[1], LegZ[1]),"\n\r"
    return

def IK(X,Y,Z,leg):
    ""
    "This function calculates the servo positions required for specific coordinates"
    #Call the translatiom function
    X = 10
    Y = 200
    Z = 0
    Theta = math.atan2(X, Y)  # Calculate Theta
    # Theta1=math.degrees(Theta1)
    print "Theta",leg,"=",math.degrees(Theta)
    ###Calculate Phi & Alpha:
    # Calculate absolute distance between Joint 2 to foot:
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
    print "Alpha",leg,"=",math.degrees(Alpha)
    c = math.asin(C * math.sin(d) / TwoToFoot)
    # print("c")
    # print(math.degrees(c))
    e = math.atan2(Reach, Z_body - Z)
    # print("e")
    # print(math.degrees(e))
    Phi = 180 - math.degrees(c) - math.degrees(e)
    Phi = math.radians(Phi)
    print "Phi  ",leg,"=",math.degrees(Phi)
    return Theta,Phi,Alpha

def Rotate(xi,yi,angle):
    ""
    "This function rotates the coordinates of a specific point around the origin by a certain angle(degrees)"
    Angle=math.radians(angle)
    xo=xi*math.cos(Angle)-yi*math.sin(Angle)
    yo=xi*math.sin(Angle)+yi*math.cos(Angle)
    return xo,yo

def RotateLeg(x,y,leg):
    ""
    "This function rotates the coordinates of a specific leg to its required position"
    Angle=math.radians((leg-1)*72)
    X=x*math.cos(Angle)-y*math.sin(Angle)
    Y=x*math.sin(Angle)+y*math.cos(Angle)
    return X,Y

def TranslateLeg(leg):
    ""
    "This function is called from the plotleg function to set the necessary offsets for plotting."
    "It calculates the offset distance from shoulder joint to robot centre"
    x=Radius*math.cos(math.radians(72*(leg-1)))
    y=Radius*math.sin(math.radians(72*(leg-1)))
    return x,y

def Vector(x,y,leg,Rot):
    ""
    "This function determines the required position of the foot given a specific vector and rotation"
    #First do the translation vector
    Angle=math.atan2(y,x)
    Magnitude=math.sqrt(x**2+y**2)                  #Calculate actual magnitude
    if (Magnitude > 0.5):                           #Limit magnitude to 0.5
        Magnitude=0.5
    NeutralX,NeutralY=RotateLeg(2.5,0,leg)
    a,b=RotateLeg(Radius,0,leg)
    NeutralX=NeutralX-a
    NeutralY=NeutralY-b
    X=NeutralX+Magnitude*math.cos(Angle)
    Y=NeutralY+Magnitude*math.sin(Angle)

    # Now the rotation part
    offsetx,offsety=TranslateLeg(leg)               #Move system origin to robot center
    X=X+offsetx
    Y=Y+offsety
    X,Y=Rotate(X,Y,-10*Rot)                              #Rotate around origin
    X=X-offsetx
    Y=Y-offsety

    return X,Y

def ConfigurePlot():
    ""
    "This function creates a 3D plot and configures axes and labels"
    fig = plt.figure()
    fig.hold
    ax = fig.add_subplot(111, projection='3d')
    blank = [0, 0]
    ax.plot(blank, blank, blank, label="Segment A", color='#FF0000')
    ax.plot(blank, blank, blank, label="Segment B", color='#00FF00')
    ax.plot(blank, blank, blank, label="Segment C", color='#0000FF')
    ax.plot(blank, blank, blank, label="Chassis", color='#AF00FF')
    ax.plot(blank, blank, blank, label="Feet zone", color='#FFAF00')

    p = Circle((0, 0), Radius, color='#AF00FF')
    ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p, z=Z_body, zdir="z")

    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis([-2.5, 2.5, -2.5, 2.5])
    ax.set_zlim3d(0, 5)
    return fig,ax

def ResetCheck():
    ""
    "This function checks each leg to see if it's in range and returns a bool for reset"

    return

####################################  Main   #####################################################

#Init Vectors
VectorX = 0                                                 # X movement vector
VectorY = 0                                                 # Y movement vector
Rot = 1                                                     # Rotation scalar
TotalTime = 5                                               #Number of time steps in simulation

for time in range (0, 5):
    time=float(time)
    Rot=time/20
    fig, ax = ConfigurePlot()


    for leg in range(1, 6):
        X,Y=Vector(VectorX,VectorY,leg,Rot)                 #Determine desired coordinates of a given leg from x,y vectors and rotation
        Theta, Phi, Alpha = IK(X, Y, Z, leg)                #Determine Servo angles for desired position
        Plotleg(Theta,Phi,Alpha,leg)                        #Plot result
        x,y=RotateLeg(2.5,0,leg)                            #Find position of little orange markers
        ax.plot([x,x],[y,y],[0,-0.05], color='#FFAF00')     #Plot little orange markers for home position

    plt.show()