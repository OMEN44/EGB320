import numpy as np
import matplotlib.pyplot as plt

obstacle_width = 0.5  # m
max_range = 2.5  # m

def attractiveField(target, phi, max_bearing_deg=90):
    target_distance, target_bearing = target
    slope = target_distance / np.radians(max_bearing_deg)
    U_att = np.maximum(0, target_distance - np.abs(phi - target_bearing) * slope)
    return U_att

target = (1.0, np.deg2rad(20))  # 1 m at +20°
phi = np.linspace(-np.pi, np.pi, 360)  # -180° to 180°

U_att = attractiveField(target, phi)

# Plot
plt.figure(figsize=(10, 4))
plt.plot(np.degrees(phi), U_att)
plt.title(r'$U_{rep}(q)$')
plt.xlabel("Angle (deg)")
plt.ylabel("Potential")
plt.grid(True)
plt.show()
