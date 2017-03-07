# Inverse Kinematics code test by plotting in 3D space
# Luan Steyn
# 2017
#import dependencies
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation
#Declare variables used

X=0                       #Result variables
Y=0
Z=0
Z_body=0.5                  #Input variables
Theta=90
Phi=0
Alpha=0
Reach=0
A=1
B=1
C=1
Radius=1
Leg1MountAngle=0
AbsDistance1=0

####################################  FUNCTIONS  ########################################################

def Plotleg (Theta,Phi,Alpha,leg):
    "This function plots a leg after the inverse kinematics are calculated"
    # Plot segment A
    LegX = [0, A * math.sin(Theta)]
    LegY = [0, A * math.cos(Theta)]
    LegZ = [Z_body, Z_body]
    ax.plot(LegX, LegY, LegZ, color='#FF0000')

    # Plot segment B
    LegX = [A * math.sin(Theta), A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta)]
    LegY = [A * math.cos(Theta), A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta)]
    LegZ = [Z_body, Z_body + B * math.cos(Phi)]
    ax.plot(LegX, LegY, LegZ, color='#00FF00')  #

    # Plot segment C
    LegX = [A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta),A * math.sin(Theta) + B * math.sin(Phi) * math.sin(Theta) + C * math.sin(Phi + Alpha - math.radians(90)) * math.sin(Theta)]
    LegY = [A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta),A * math.cos(Theta) + B * math.sin(Phi) * math.cos(Theta) + C * math.sin(Phi + Alpha - math.radians(90)) * math.cos(Theta)]
    LegZ = [Z_body + B * math.cos(Phi), Z_body + B * math.cos(Phi) + C * math.cos(Phi + Alpha - math.radians(90))]
    ax.plot(LegX, LegY, LegZ, color='#0000FF')  #
    print(LegX[1], LegY[1], LegZ[1])
    return

def IK(X,Y,Z,leg):
    Theta = math.atan2(X, Y)  # Calculate Theta
    # Theta1=math.degrees(Theta1)
    print("Theta")
    print(math.degrees(Theta))

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
    print("Alpha")
    print(math.degrees(Alpha))
    c = math.asin(C * math.sin(d) / TwoToFoot)
    # print("c")
    # print(math.degrees(c))
    e = math.atan2(Reach, Z_body - Z)
    # print("e")
    # print(math.degrees(e))
    Phi = 180 - math.degrees(c) - math.degrees(e)
    Phi = math.radians(Phi)
    print("Phi")
    print(math.degrees(Phi))
    return Theta,Phi,Alpha

####################################  Main   #####################################################



#3D Plot config
fig =plt.figure()
ax=fig.add_subplot(111,projection='3d')
blank=[0,0]
ax.plot(blank,blank,blank,label="Segment A", color='#FF0000')
ax.plot(blank,blank,blank,label="Segment B", color='#00FF00')
ax.plot(blank,blank,blank,label="Segment C", color='#0000FF')

#Leg 1
X=1.5
Y=1.5
Theta,Phi,Alpha=IK(X,Y,Z,1)
Plotleg(Theta,Phi,Alpha,1)

#Leg 2
X=-1.5
Y=1.5
Theta,Phi,Alpha=IK(X,Y,Z,1)
Plotleg(Theta,Phi,Alpha,1)

#Leg 3
X=-1.5
Y=-1.5
Theta,Phi,Alpha=IK(X,Y,Z,1)
Plotleg(Theta,Phi,Alpha,1)

#Leg 4
X=1.5
Y=-1.5
Theta,Phi,Alpha=IK(X,Y,Z,1)
Plotleg(Theta,Phi,Alpha,1)

#Leg 5
X=2.5
Y=0
Theta,Phi,Alpha=IK(X,Y,Z,1)
Plotleg(Theta,Phi,Alpha,1)

# 2D Plot
# plt.plot(LegX,LegZ);
# plt.axis([-1,3, -1, 3])
# plt.show()


ax.legend()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.axis([-2.5,2.5,-2.5,2.5])
ax.set_zlim3d(0,5)
plt.show()