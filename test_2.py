import matplotlib.pyplot as plt
import numpy as np

# Données
robot_pos = (0, 0)
robot_angle = 359.9999996678939


# points = [
#     {"position": (9, 39), "point_angle": 77.00538320808349, "relative_angle": 9.449086858025993},
#     {"position": (-14, 36), "point_angle": 111.25050550713324, "relative_angle": 24.79603544102376},
#     {"position": (-21, 16), "point_angle": 142.6960517220166, "relative_angle": 56.24158165590711},
#     {"position": (7, 12), "point_angle": 59.74356283647074, "relative_angle": 26.710907229638742},
#     {"position": (5, 5), "point_angle": 45.0, "relative_angle": 41.45447006610948},
# ]

points = [
    {"position": (33, 2), "point_angle": -3.0810606338076862, "relative_angle": 172.8255286809626},
    {"position": (-7, -21), "point_angle": 1.2490457723982544, "relative_angle": 168.49542227475666},
    {"position": (29, -16), "point_angle": 2.6374266921106404, "relative_angle": 167.1070413550443},
]




# Configuration du graphique
plt.figure(figsize=(8, 8))
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)

# Tracer la position du robot
plt.scatter(*robot_pos, color='blue', label="Robot", zorder=5)

# Tracer les points et les angles
for point in points:
    pos = point["position"]
    plt.scatter(*pos, color='red', label="Point cible" if points.index(point) == 0 else "", zorder=5)

    # Tracer une ligne entre le robot et le point
    plt.plot([robot_pos[0], pos[0]], [robot_pos[1], pos[1]], color='gray', linestyle='--', zorder=1)

    # Calcul et tracé des angles
    angle_rad = np.deg2rad(point["point_angle"])
    plt.quiver(robot_pos[0], robot_pos[1],
               np.cos(angle_rad), np.sin(angle_rad),
               angles='xy', scale_units='xy', scale=1, color='green', label="Angle cible" if points.index(point) == 0 else "")

# Ajouter des légendes et détails
plt.legend()
plt.title("Visualisation des positions et angles")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis('equal')

# Afficher le graphique
plt.show()
