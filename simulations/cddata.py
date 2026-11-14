import matplotlib.pyplot as plt
import numpy as np

alpha_vals = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0])
rho_vals = np.array([1.170, 1.170, 1.170, 1.170, 1.170, 1.170, 1.170, 1.170, 1.170, 1.170, 1.170, 1.156, 1.113])
velo_vals = np.array([144.29, 144.29, 144.29, 144.29, 144.29, 144.29, 144.29, 144.29, 144.29, 144.29, 144.29, 130.70, 86.01])
cda_vals = np.array([0.0082223, 0.0083765, 0.0087967, 0.0092316, 0.0097345, 0.0102216, 0.0106441, 0.0110802, 0.0117885, 0.0126849, 0.0135710, 0.0136017, 0.0137743])

# rng = np.random.default_rng(42)
# x = rng.normal(size=100)
# y = rng.normal(size=100)
# z = x ** 2 + y ** 2

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

# scatter = ax.scatter(x, y, z, c=z, cmap="viridis", s=40)

scatter = ax.scatter3D(alpha_vals, rho_vals, cda_vals)

# ax.set_xlabel("X Axis")
# ax.set_ylabel("Y Axis")
# ax.set_zlabel("Z Axis")
# ax.set_title("3D Scatter Plot")
# fig.colorbar(scatter, ax=ax, shrink=0.6, label="Z Value")

ax.set_xlabel("Deployment Level")
ax.set_ylabel("Velocity")
ax.set_zlabel("Drag Coefficient")

plt.show()
