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

X1=1#Result variables
Y1=1
Z1=0
Z_body=0.5                #Input variables
Theta1=90
Phi1=0
Alpha1=0
Reach1=0
A=1
B=1
C=1
Radius=1
Leg1MountAngle=0
AbsDistance1=0

####################################CALCULATIONS#####################################################

#Calculate Theta
Theta1 = math.atan2(X1, Y1)
Theta1=math.degrees(Theta1)
print("Theta")
print(Theta1)

#Calculate Phi & Alpha
#AbsDistance1=math.sqrt(X1**2+Y1**2+(Z_body-Z1)**2)          #Absolute Distance from Joint 1 to foot
Reach1=math.sqrt(X1**2+Y1**2)                               #Horizontal Distance from Joint 1 to foot
#Calculate absolute distance between Joint 2 to foot:
x=A*math.sin(math.radians(Theta1))
y=A*math.cos(math.radians(Theta1))
z=Z_body
TwoToFoot=math.sqrt((X1-x)**2+(Y1-y)**2+(Z1-z)**2)
print("TwoToFoot")
print(TwoToFoot)
x=(B**2+C**2-TwoToFoot**2)/(2*B*C)
d=math.acos(x)
# print("d")
# print(math.degrees(d))
Alpha1=270-math.degrees(d)
print("Alpha")
print(Alpha1)
c=math.asin(C*math.sin(d)/TwoToFoot)
# print("c")
# print(math.degrees(c))
e=math.atan2(Reach1,Z_body-Z1)
# print("e")
# print(math.degrees(e))
Phi1=180-math.degrees(c)-math.degrees(e)
print("Phi")
print(Phi1)

#Plot legs
# LegX=[0,A,A+B*math.sin(math.radians(Phi1)),A+B*math.sin(math.radians(Phi1))+C*math.sin(math.radians(Phi1)+math.radians(Alpha1)-math.radians(90))]
# LegY=[0,A*math.cos(math.radians(Theta1)),A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1)),A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1))+C*math.cos(math.radians(Theta1))]
# LegZ=[Z_body,Z_body,Z_body+B*math.cos(math.radians(Phi1)),Z_body+B*math.cos(math.radians(Phi1))+C*math.cos(math.radians(Phi1)+math.radians(Alpha1)-math.radians(90))]

#3D Plot
fig =plt.figure()
ax=fig.add_subplot(111,projection='3d')


#Plot segment A
LegX=[0,A*math.sin(math.radians(Theta1))]
LegY=[0,A*math.cos(math.radians(Theta1))]
LegZ=[Z_body,Z_body]
ax.plot(LegX,LegY,LegZ,label='A',color='#FF0000')


#Plot segment B
LegX=[A*math.sin(math.radians(Theta1)),A*math.sin(math.radians(Theta1))+B*math.sin(math.radians(Phi1))*math.sin(math.radians(Theta1))]
LegY=[A*math.cos(math.radians(Theta1)),A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1))]
LegZ=[Z_body,Z_body+B*math.cos(math.radians(Phi1))]
ax.plot(LegX,LegY,LegZ,label='B',color='#00FF00')#

#Plot segment C
LegX=[A*math.sin(math.radians(Theta1))+B*math.sin(math.radians(Phi1))*math.sin(math.radians(Theta1)),A*math.sin(math.radians(Theta1))+B*math.sin(math.radians(Phi1))*math.sin(math.radians(Theta1))+C*math.sin(math.radians(Phi1)+math.radians(Alpha1)-math.radians(90))*math.sin(math.radians(Theta1))]
LegY=[A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1)),A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1))+C*math.cos(math.radians(Theta1))]
LegZ=[Z_body+B*math.cos(math.radians(Phi1)),Z_body+B*math.cos(math.radians(Phi1))+C*math.cos(math.radians(Phi1)+math.radians(Alpha1)-math.radians(90))]
ax.plot(LegX,LegY,LegZ,label='C',color='#0000FF')#

# 2D Plot
# plt.plot(LegX,LegZ);
# plt.axis([-1,3, -1, 3])
# plt.show()


ax.legend()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.axis([-3,3,-3,3])
plt.show()