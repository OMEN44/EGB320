import os
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


def attractiveField(target, phi, max_bearing_deg=90):
    target_distance, target_bearing = target
    slope = target_distance / np.radians(max_bearing_deg)
    U_att = np.maximum(0, target_distance - np.abs(phi - target_bearing) * slope)
    return U_att


# Example usage
obstacles = [
    (2.0, np.deg2rad(45)),   # 2 m at +45°
    (1.0, np.deg2rad(-30))   # 1 m at -30°
]
target = (1.0, np.deg2rad(45))  # 1 m at +20°

# Define angular space
phi = np.linspace(-np.pi, np.pi, 360)  # -180° to 180°

# Compute fields
U_att = attractiveField(target, phi)
U_rep = repulsiveField(obstacles, phi)
# U_total = U_att + U_rep
U_total = U_att - U_rep


# Plot
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axs[0].plot(np.degrees(phi), U_att)
axs[0].set_title(r'$U_{att}(q)$')

axs[1].plot(np.degrees(phi), U_rep)
axs[1].set_title(r'$U_{rep}(q)$')

axs[2].plot(np.degrees(phi), U_total)
axs[2].set_title(r'$U_{att}(q) + U_{rep}(q)$')

for ax in axs:
    ax.grid(True)
    ax.set_ylabel("Potential")
axs[2].set_xlabel("Angle (deg)")

plt.tight_layout()
plt.show()

best_index = np.argmax(U_total)
best_bearing = phi[best_index]   # in radians
best_bearing_deg = np.degrees(best_bearing)

print("Best heading (rad):", best_bearing)
print("Best heading (deg):", best_bearing_deg)