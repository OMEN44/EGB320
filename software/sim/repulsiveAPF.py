import numpy as np
import matplotlib.pyplot as plt

obstacle_width = 0.5  # m
max_range = 2.5  # m

def repulsiveField(obstacleList, phi):
    U_rep = np.zeros_like(phi)

    for obs in obstacleList:
        if obs:
            obs_distance, obs_bearing = obs
            if obs_distance <= 0 or abs(obs_bearing) > np.pi/2:
                continue

            # Angular half-width of obstacle
            dphi = np.arcsin((obstacle_width/2) / obs_distance) if obs_distance > (obstacle_width/2) else np.pi/2

            # Square repulsive field: constant height across angular span
            mask = (phi >= (obs_bearing - dphi)) & (phi <= (obs_bearing + dphi))
            U_rep[mask] = np.maximum(U_rep[mask], (1.0 / obs_distance))  # stronger if closer

    return U_rep

# Example obstacles
obstacles = [
    (2.0, np.deg2rad(45)),   # 2 m at +45°
    (1.0, np.deg2rad(-30))   # 1 m at -30°
]

# Define angular space
phi = np.linspace(-np.pi, np.pi, 360)  # -180° to 180°

# Compute repulsive field
U_rep = repulsiveField(obstacles, phi)

# Plot only repulsive field
plt.figure(figsize=(10, 4))
plt.plot(np.degrees(phi), U_rep)
plt.title(r'$U_{rep}(q)$')
plt.xlabel("Angle (deg)")
plt.ylabel("Potential")
plt.grid(True)
plt.show()