import matplotlib.pyplot as plt
import numpy as np

# Load data from file
data = np.loadtxt('../build/tcp_positions.txt')

# Extract time and positions
time = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]


# Plot the TCP coordinates over time
plt.figure()
plt.plot(time, x, label='X')
plt.plot(time, y, label='Y')
plt.plot(time, z, label='Z')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.title('TCP Position Over Time')
plt.legend()
plt.grid()
plt.show()