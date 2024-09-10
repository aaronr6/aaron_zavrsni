import matplotlib.pyplot as plt
import numpy as np

# Load data from file
data = np.loadtxt('../build/tcp_positions.txt')

# Extract time and positions
time = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]
label = ['X', 'Y', 'Z', 'X_d', 'Y_d', 'Z_d']


# Plot the TCP coordinates over time
for i in range(1,4):
    plt.figure()
    plt.plot(time, data[:, i], label=label[i-1])
    plt.plot(time, data[:, (i+3)], label=label[i+2])
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.title('TCP Position Over Time')
    plt.legend()
    plt.grid()
    plt.show()