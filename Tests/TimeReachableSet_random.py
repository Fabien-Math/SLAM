import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from tqdm import tqdm

max_left_ang_speed = 1
max_right_ang_speed = 1
R = 0.04
d = 0.15
instants = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

possibilites = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]


N = 10000
X, Y = [0]*N*len(instants), [0]*N*len(instants)
for j, instant in enumerate(instants):
	time = np.linspace(0, 1, instant)
	dt = 1/instant

	left_ang_speed = 0
	right_ang_speed = 0
	for i in tqdm(range(N)):

		x1, y1, theta = 0, 0, 0
		for t in time:
			# left_ang_speed = np.random.randint(-1, 2)
			# right_ang_speed = np.random.randint(-1, 2)

			match left_ang_speed:
				case -1:
					left_ang_speed = np.random.randint(-1, 1)
				case 0:
					left_ang_speed = np.random.randint(-1, 2)
				case 1:
					left_ang_speed = np.random.randint(0, 2)

			match right_ang_speed:
				case -1:
					right_ang_speed = np.random.randint(-1, 1)
				case 0:
					right_ang_speed = np.random.randint(-1, 2)
				case 1:
					right_ang_speed = np.random.randint(0, 2)
			
			theta += R/(2 * d) * (right_ang_speed - left_ang_speed) * dt
			x1 += R/2 * (left_ang_speed + right_ang_speed) * np.cos(theta) * dt
			y1 += R/2 * (left_ang_speed + right_ang_speed) * np.sin(theta) * dt


		X[j*N + i] = x1/instant
		Y[j*N + i] = y1/instant

points = np.array([(xi, yi) for xi, yi in zip(X, Y)])
hull = ConvexHull(points)
valid_ids = np.append(hull.vertices, hull.vertices[0])

plt.figure('random', layout='constrained')
plt.scatter(X, Y)
plt.title("Time-reachable set using randomness")
plt.xlabel("x (mu)")
plt.ylabel("y (mu)")
plt.plot(points[valid_ids, 0], points[valid_ids, 1], color="C1")
plt.axis("equal")
plt.show()