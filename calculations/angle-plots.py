import spinmob
import matplotlib.pyplot as plt
import numpy as np

ax = plt.figure(1).add_subplot(projection='3d')

# Prepare arrays x, y, z
theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
t = np.linspace(-1, 1, 100)
x =   t; y = 0*t; z = x;   ax.plot(x, y, z)
x = 0*t; y =   t; z = y;   ax.plot(x, y, z)
x =   t; y =   t; z = y;   ax.plot(x, y, z)
x =   t; y =  -t; z = 0*t; ax.plot(x, y, z)


ax.legend()

plt.show()