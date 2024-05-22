import matplotlib.pyplot as plt
import numpy as np

# Read the joint states from the file
data = np.loadtxt('../build/joint_states.txt')

# Create a time array
time = np.arange(0, data.shape[0])

# Plot each joint state
for i in range(data.shape[1]):
    plt.plot(time, data[:, i], label=f'Joint {i+1}')

plt.xlabel('Time')
plt.ylabel('Joint State')
plt.title('Joint States over Time')
plt.legend()
plt.show()