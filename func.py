import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(1, 2, 10000)
y1 = 1/x
y2 = np.tanh(x)

z = lambda x : 1/x - np.tanh(x)


plt.plot(x, y1)
plt.plot(x, y2)
plt.plot(x, z(x))
plt.axis([1, 2, -1, 5])
plt.show()