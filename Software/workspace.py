# %% Setup

# general imports
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import numpy as np
from tqdm import tqdm

# Custom imports
import kinematics as kin

# %% Run fk 
dh = [[0, 2.5, 0, np.pi/2],
          [0, 0, 2.5, 0],
          [np.pi/3, 0, 2.5, 0]]

Arm = kin.SerialArm(dh)
steps = 25
Range = [np.linspace(0, np.pi, steps) for _ in range(Arm.n)]

loop = tqdm(total = steps**3)

# Append ee position in reachable workspace to points list
points = []
for i in Range[0]:
    for j in Range[1]:
        for k in Range[2]:
            point = Arm.fk(q = [i, j, k])[:3, 3]
            points.append(point)
            loop.update(1)

loop.close()

# %% Visualize

print(points)

# %%
