import numpy as np

data = np.loadtxt('Graph1.txt', delimiter=',', dtype='Float64', skiprows=1)
x = np.array(data[:, 1])
x_std = np.std(x)
print(x_std)

data = np.loadtxt('Graph2.txt', delimiter=',', dtype='Float64', skiprows=1)
x_a = np.array(data[:, 1])
x_a_std = np.std(x_a)
print(x_a_std)

