import numpy as np
import math
import time  # Add if not present

obstacle_width = 0.15  # meters

# ---------------------------
# Potential fields
# ---------------------------
# def repulsiveField(obstacleList, phi=np.linspace(-np.pi, np.pi, 360)):
#     U_rep = np.zeros_like(phi)
#     if not obstacleList:
#         return U_rep
    
#     for obs in obstacleList:
#         obs_distance, obs_bearing = obs
#         if obs_distance <= 0 or abs(obs_bearing) > np.pi/2:
#             continue

#         dphi = np.arcsin((obstacle_width / 2) / obs_distance) if obs_distance > (obstacle_width / 2) else np.pi/2
#         mask = (phi >= (obs_bearing - dphi)) & (phi <= (obs_bearing + dphi))
#         k_rep = 10  # or higher
#         U_rep[mask] = np.maximum(U_rep[mask], k_rep / obs_distance)
#     return U_rep

# def attractiveField(target, phi=np.linspace(-np.pi, np.pi, 360), max_bearing_deg=90):
#     if target is None:
#         return np.zeros_like(phi)
#     target_distance, target_bearing = target
#     slope = target_distance / np.radians(max_bearing_deg)
#     U_att = np.maximum(0.0, target_distance - np.abs(phi - target_bearing) * slope)
#     return U_att

def repulsiveField(obstacleList, phi=np.linspace(-np.pi, np.pi, 360), obstacle_width=0.3):
    U_rep = np.zeros_like(phi)
    if not obstacleList:
        return U_rep

    for obs in obstacleList:
        # --- Robust sanity check ---
        if not isinstance(obs, (list, tuple)) or len(obs) != 2:
            continue

        obs_distance, obs_bearing = obs
        if not isinstance(obs_distance, (int, float)) or not isinstance(obs_bearing, (int, float)):
            continue
        if obs_distance <= 0 or abs(obs_bearing) > np.pi / 2:
            continue

        dphi = np.arcsin((obstacle_width / 2) / obs_distance) if obs_distance > (obstacle_width / 2) else np.pi / 2
        mask = (phi >= (obs_bearing - dphi)) & (phi <= (obs_bearing + dphi))
        k_rep = 20

        # Smooth cosine decay for repulsion
        decay = np.cos((phi[mask] - obs_bearing) / dphi * (np.pi / 2)) ** 2
        U_rep[mask] = np.maximum(U_rep[mask], k_rep * decay / obs_distance**2)

    return U_rep



# Attractive field towards target
def attractiveField(target, phi=np.linspace(-np.pi, np.pi, 360), max_bearing_deg=90):
    if target is None:
        return np.zeros_like(phi)
    target_distance, target_bearing = target
    slope = target_distance / np.radians(max_bearing_deg)
    U_att = np.maximum(0.0, target_distance - np.abs(phi - target_bearing) * slope)
    return U_att

def bestBearing(U_att, U_rep, phi=np.linspace(-np.pi, np.pi, 360)):
    U_total = U_att - U_rep
    if not np.isfinite(U_total).all():
        return None
    if np.allclose(U_total, 0.0):
        return None
    best_index = np.argmax(U_total)
    return phi[best_index]   # radians

def angle_wrap(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2*np.pi) - np.pi