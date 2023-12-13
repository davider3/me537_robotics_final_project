# %% Setup

# general imports
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import numpy as np
from tqdm import tqdm
import time
import os
from PIL import Image
import glob

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

# %% Create images

# Unpack the arrays into separate lists for x, y, and z coordinates
x, y, z = zip(*points)

# Create a smaller 3D plot (adjust the width and height as needed)
fig = plt.figure(figsize=(6, 4))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot the points
scatter = ax.scatter(x, y, z, s=.5)

# Set labels for each axis
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

output_folder = "C:/Users/drein/Documents/Classes/ME_537_robotics/me537_robotics_final_project/Software/Images/new_gif"

steps = 360
loop = tqdm(total=steps)
# Allow interactive rotation
for i in range(steps):
    ax.view_init(azim=i)
    plt.savefig('C:/Users/drein/Documents/Classes/ME_537_robotics/me537_robotics_final_project/Software/Images/new_gif/{0:03}.png'.format(i))
    loop.update(1)

loop.close()

# %% Create GIF
frames = [Image.open(image) for image in glob.glob(f'{output_folder}/*.png')]
frame_one = frames[0]
frame_one.save("newest.gif", format="GIF", append_images=frames,
                save_all=True, duration=1, loop=0)

# %%
