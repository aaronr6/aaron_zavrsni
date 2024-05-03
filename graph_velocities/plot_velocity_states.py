import matplotlib.pyplot as plt
import numpy as np

# Read the velocity states from the file
data = np.loadtxt('../build/velocity_states.txt')

# Create a time array
time = np.arange(0, data.shape[0])

# Plot each velocity state
for i in [1, 2, 3]:
    plt.plot(time, data[:, i], label=f'Velocity {i}')

plt.xlabel('Time')
plt.ylabel('Velocity State')
plt.title('Velocity States over Time')
plt.legend()
plt.show()