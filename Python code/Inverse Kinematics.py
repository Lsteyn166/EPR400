# Inverse Kinematics code test by plotting in 3D space
# Luan Steyn
# 2017
#import dependencies
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#Declare variables used

X1=0                     #Result variables
Y1=0
Z1=0
Z_body=1                #Input variables
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

#Calculations
if (Y1!=0 & X1!=0):
    Theta1 = math.atan2(X1, Y1)
else:
    Theta1 = math.radians(90)
Theta1=math.degrees(Theta1)
print("Theta")
print(Theta1)
AbsDistance1=math.sqrt(X1**2+Y1**2+(Z_body-Z1)**2)        #Absolute Distance from Joint 1 to foot
Reach1=math.sqrt(X1**2+Y1**2)
# print("Abs")
# print(AbsDistance1)
x=(B**2+C**2-AbsDistance1**2)/(2*B*C)
d=math.acos(x)
# print("d")
# print(math.degrees(d))
Alpha1=270-math.degrees(d)
print("Alpha")
print(Alpha1)
c=math.asin(C*math.sin(d)/AbsDistance1)
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
LegX=[0,A]
LegY=[0,A*math.cos(math.radians(Theta1))]
LegZ=[Z_body,Z_body]
ax.plot(LegX,LegY,LegZ,label='A',color='#FF0000')
print(LegY)

#Plot segment B
LegX=[A,A+B*math.sin(math.radians(Phi1))]
LegY=[A*math.cos(math.radians(Theta1)),A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1))]
LegZ=[Z_body,Z_body+B*math.cos(math.radians(Phi1))]
ax.plot(LegX,LegY,LegZ,label='B',color='#00FF00')#
print(LegY)

#Plot segment C
LegX=[A+B*math.sin(math.radians(Phi1)),A+B*math.sin(math.radians(Phi1))+C*math.sin(math.radians(Phi1)+math.radians(Alpha1)-math.radians(90))]
LegY=[A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1)),A*math.cos(math.radians(Theta1))+B*math.cos(math.radians(Theta1))+C*math.cos(math.radians(Theta1))]
LegZ=[Z_body+B*math.cos(math.radians(Phi1)),Z_body+B*math.cos(math.radians(Phi1))+C*math.cos(math.radians(Phi1)+math.radians(Alpha1)-math.radians(90))]
ax.plot(LegX,LegY,LegZ,label='C',color='#0000FF')#
print(LegY)
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