# %% Setup

# general imports
import time
import inspect
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout
import numpy as np

# Custom imports
import kinematics as kin
from visualization import VizScene
import transforms as tr

# %% Run fk 
dh = [[0, 2.5, 0, np.pi/2],
          [0, 0, 2.5, 0],
          [np.pi/3, 0, 2.5, 0]]

Arm = kin.SerialArm(dh)
steps = 10
Range = [np.linspace(0, np.pi, steps) for _ in range(Arm.n)]

# Append ee position in reachable workspace to points list
points = []
for i in range(steps):
    for j in range(steps):
        for k in range(steps):
            points.append(Arm.fk(q = [i, j, k])[:3, 3])

# Visualize the robot arm
viz = VizScene()
viz.add_arm(Arm)
viz.hold()
            
# %%

# %%
