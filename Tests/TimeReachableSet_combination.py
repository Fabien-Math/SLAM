import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from itertools import product
from tqdm import tqdm


def generate_combinations(N):
	possibilities = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
	
	# Generate all possible combinations of length N
	combinations = list(product(possibilities, repeat=N))
	
	return combinations

def compute_range(nt, R, d, tf):
	left_right = generate_combinations(nt)
	if tf:
		dt = tf / nt
	else:
		dt = ((d / R) * 0.5* np.pi) / nt

	X, Y = [], []
	for k in tqdm(range(len(left_right))):
		lr = left_right[k]
		x1, y1, theta = 0, 0, 0
		for i in range(nt):
			left_ang_speed, right_ang_speed = lr[i]
			theta += R/(2 * d) * (right_ang_speed - left_ang_speed) * dt
			x1 += R/2 * (left_ang_speed + right_ang_speed) * np.cos(theta) * dt
			y1 += R/2 * (left_ang_speed + right_ang_speed) * np.sin(theta) * dt

		X.append(x1)
		Y.append(y1)

	return np.array(X), np.array(Y)



max_left_ang_speed = 1	# rad/s
max_right_ang_speed = 1	# rad/s
R = 0.04				# m
d = 0.15				# m


nt = 6
tf = 1
X, Y = compute_range(nt, R, d, tf)
maskX = X > 0
X_tronc = X[maskX]
Y_tronc = Y[maskX]
maskY = Y_tronc >= 0
X_tronc = X_tronc[maskY]
Y_tronc = Y_tronc[maskY]
plt.figure(f"Available locations for {nt} commands in {tf}s", layout="constrained", figsize=(19.2, 10.8))
plt.scatter(X, Y, s=10)

points = np.array([(xi, yi) for xi, yi in zip(X, Y)])
hull = ConvexHull(points)
valid_ids = np.append(hull.vertices, hull.vertices[0])

plt.plot(points[valid_ids, 0], points[valid_ids, 1], color="C1")
plt.title("Time-reachable set computing all combinations")
plt.xlabel("x (mu)")
plt.ylabel("y (mu)")
plt.axis("equal")
plt.show()