import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
R0=np.array([0,0.098,0.1])
R1=np.array([R0[0]-0.00165,R0[1]-0.02663,R0[2]+0.00014])
R2=np.array([R1[0]+0.12955,R1[1]+0.02063,R1[2]+0.0033])
R3=np.array([R2[0]-0.02744,R2[1],R2[2]-0.00014])
R4=np.array([R3[0]+0.05112,R3[1]+0.00562,R3[2]+0.00152])

points = np.array([R0, R1, R2, R3, R4])


fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.plot(0,0,0,'o',label='torso')
ax.plot(R0[0],R0[1],R0[2],'o',label=names[0])
ax.plot(R1[0],R1[1],R1[2],'o',label=names[1])
ax.plot(R2[0],R2[1],R2[2],'o',label=names[2])
ax.plot(R3[0],R3[1],R3[2],'o',label=names[3])
ax.plot(R4[0],R4[1],R4[2],'o',label=names[4])
ax.plot(points[0:2, 0], points[0:2, 1], points[0:2, 2], 'g-')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()
