#!/usr/bin/env python
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

NO_OF_SAMPLES = 9000
LINE_WIDTH = 10

our = np.genfromtxt("../our_exploration2.txt", delimiter=",")
wander = np.genfromtxt("../wanderer_exploration2.txt", delimiter=",")

matplotlib.rcParams.update({'font.size': 30})

plt.hold(True)
plt.plot(our[0:NO_OF_SAMPLES, 0]/60., our[0:NO_OF_SAMPLES, 1]*0.01, label='Reinforced Network', linewidth=LINE_WIDTH)
plt.plot(wander[0:NO_OF_SAMPLES, 0]/60., wander[0:NO_OF_SAMPLES, 1]*0.01, label='Wanderer Algorithm', linewidth=LINE_WIDTH)
plt.xlabel("Time (mins)", fontsize=35)
plt.ylabel("Explored area (m^2)", fontsize=35)
plt.legend(loc=9, prop={'size':40})
plt.grid()
plt.show()