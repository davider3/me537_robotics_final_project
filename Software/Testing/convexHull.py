# %% imports
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# %% Code
# Generate a large random set of 3D points
np.random.seed(42)
points = np.random.rand(1000, 3)

# Compute the convex hull
hull = ConvexHull(points)

# Plot the original points and the convex hull
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the original points
# ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="b", marker="o", label="Original Points")

# Plot the convex hull
ax.plot_trisurf(points[:, 0], points[:, 1], points[:, 2], triangles=hull.simplices, color="r", alpha=0.2, label="Convex Hull")

# Set labels and legend
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()

# Show the plot
plt.show()

# %%
