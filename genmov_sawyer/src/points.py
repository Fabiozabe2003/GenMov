import numpy as np
import matplotlib.pyplot as plt
from lab4functions import *

# Load the array from the txt file
points = np.loadtxt('/home/andres/momask-codes/right_hand_points.txt', dtype=float)

it = np.linspace(0, len(points)-1, len(points))  # Array with 100 points from 0 to 10

x=points[:,0]; y=points[:,1]; z=points[:,2]

plt.figure(1)
plt.plot(it, x, label='X actual')
plt.plot(it,y, label='X deseado')

# Step 5: Customize the plot (optional)
plt.title('X actual vs X deseado')
plt.xlabel('Iteration')
plt.ylabel('X axis')
plt.legend()

# Display plots
plt.show()
