import numpy as np
import make_body_plot as m
import matplotlib.pyplot as plt

numPts = 1800
vels = np.load("limbVels.npy")
pos = np.load("limbPos.npy")
m.make_graded_limb_plot(pos[:numPts],vels[:numPts],0,20)
plt.show()

