"""
====================
3D plots as subplots
====================

Demonstrate including 3D plots as subplots.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D, get_test_data
from matplotlib import cm
import numpy as np
import math
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
try:
    from Robot_paths_lib import *
    from Robot_draw_lib import *
except ImportError:
    raise

plotter = Plot_robot(title="Ranking visualization")
plotter.ax = plotter.fig.add_subplot(projection='3d')

map_size = 100
map_resolution = 2
angle_resolution = math.pi/90

# plot a 3D surface like in the example mplot3d/surface3d_demo
angle = np.arange(0.001, math.pi, angle_resolution)
distance = np.arange(1, map_size, map_resolution)

X = angle
Y = distance
X, Y = np.meshgrid(X, Y)
Angle = X/math.pi
Distance = Y/map_size
ranking_score = 0.1/abs(Angle) + 0.9/abs(Distance)
Z =np.log(ranking_score)

surf = plotter.ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
#ax.set_zlim(-1.01, 1.01)
plotter.fig.colorbar(surf, shrink=0.5)

plotter.ax.set_xlabel('Angle')
plotter.ax.set_ylabel('Distance')
plotter.ax.set_zlabel('Ranking score')

plotter.show()
