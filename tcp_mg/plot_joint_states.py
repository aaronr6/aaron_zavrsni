import matplotlib.pyplot as plt
import numpy as np

# Read the joint states from the file
data = np.loadtxt('../build/joint_states.txt')

# Create a time array
time = np.arange(0, data.shape[0])

# Plot the 13th, 14th, and 15th joint states
for i in [12, 13, 14]:
    plt.plot(time, data[:, i], label=f'Joint {i+1}')

plt.xlabel('Time')
plt.ylabel('Cartesian Coordinate')
plt.title('Cartesian pose over Time')
plt.legend()
plt.show()