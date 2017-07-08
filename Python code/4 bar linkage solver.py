#############################################################
# Author: LSteyn                                            #
# Program for solving 4 bar linkage lengths for given angles#
#############################################################

#                    _____a2_____
#                   /           /
#                  /           /
#                 /           /
#              a3/           /a1
#               /           /
#              /           /
#             /leg________/servo
#                  a4

import math
import matplotlib.pyplot as plt

#Define angles for 3 given positions
servo1 = 30.0
leg1 = 90.0
servo2 = 180.0
leg2 = 170.0
servo3 = (servo2+servo1)/2
leg3 = (leg2+leg1)/2

#Define ground linkage length
a4 = 28.5   #in mm
# a4 = 30   #in mm

#Calculations
x1 = math.cos(math.radians(servo1)) - math.cos(math.radians(servo2))
x2 = math.cos(math.radians(leg1)) - math.cos(math.radians(leg2))
x3 = math.cos(math.radians(servo1-leg1)) - math.cos(math.radians(servo2 - leg2))
x4 = math.cos(math.radians(servo1)) - math.cos(math.radians(servo3))
x5 = math.cos(math.radians(leg1)) - math.cos(math.radians(leg3))
x6 = math.cos(math.radians(servo1-leg1)) - math.cos(math.radians(servo3 - leg3))

k1 = (x2*x6-x3*x5)/(x2*x4-x1*x5)
k2 = (x1*x6-x3*x4)/(x2*x4-x1*x5)
k3 = math.cos(math.radians(servo1-leg1)) - k1*math.cos(math.radians(servo1)) + k2*math.cos(math.radians(leg1))

a1 = a4/k2
a3 = a4/k1
a2 = math.sqrt(a1**2+a3**2+a4**2-2*a1*a3*k3)
print "a1=",math.fabs(a1)
print "a2=",math.fabs(a2)
print "a3=",math.fabs(a3)
print "a4=",a4
ratio = (servo2-servo1)/(leg2-leg1)
print "Ratio = ",ratio

#Torque calculation
Ts = 2      # in kg-cm


# Plots
fig = plt.figure()
ax = fig.add_subplot(111)
x = [0,a4,a4+a1*math.cos(math.radians(servo1)),a3*math.cos(math.radians(leg1)),0]
y = [0,0,a1*math.sin(math.radians(servo1)),a3*math.sin(math.radians(leg1)),0]
ax.plot(x,y,"red")
x = [0,a4,a4+a1*math.cos(math.radians(servo2)),a3*math.cos(math.radians(leg2)),0]
y = [0,0,a1*math.sin(math.radians(servo2)),a3*math.sin(math.radians(leg2)),0]
ax.plot(x,y,"green")
x = [0,a4,a4+a1*math.cos(math.radians(servo3)),a3*math.cos(math.radians(leg3)),0]
y = [0,0,a1*math.sin(math.radians(servo3)),a3*math.sin(math.radians(leg3)),0]
ax.plot(x,y,"blue")
ax.plot([0,0],[-1,-1])
plt.show()



########################################################################################################################
# Results
# Leg A:
# servo1 = 30.0
# leg1 = 90.0
# servo2 = 180.0
# leg2 = 170.0
#
# Leg C:
# servo1 = 0.0
# leg1 = 45.0
# servo2 = 150.0
# leg2 = 135.0
