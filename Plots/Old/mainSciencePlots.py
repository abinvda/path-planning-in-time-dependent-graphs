import matplotlib.pyplot as plt
import numpy as np

import scienceplots
plt.style.use('ieee')
# plt.style.use(['science', 'ieee'])

opt = np.random.randint(600, size=(200))
gree = opt*(1+np.random.rand(200))

# plt.plot(opt, gree, 'o')
plt.scatter(opt, gree, s=10)
plt.plot(gree, gree)
# plt.axis('equal')
plt.xlim([0, 1120])
plt.ylim([0, 1120])
plt.xlabel("Optimal Path Duration (min)")
plt.ylabel("Greedy Path Duration (min)")

plt.show()