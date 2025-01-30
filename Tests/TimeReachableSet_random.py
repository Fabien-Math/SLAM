import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d

max_left_ang_speed = 1
max_right_ang_speed = 1
R = 0.04
d = 0.15
instants = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

possibilites = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
left_right = np.concatenate([[[p, pos] for p in possibilites] for pos in possibilites])
left_right = np.concatenate([[[[p, pos, poss] for p in possibilites] for pos in possibilites] for poss in possibilites])

print(len(left_right))

N = 1000
X, Y = [0]*N*len(instants), [0]*N*len(instants)
for j, instant in enumerate(instants):
	time = np.linspace(0, 1, instant)

	left_ang_speed = 0
	right_ang_speed = 0
	for i in range(N):

		x1, y1, theta = 0, 0, 0
		for t in time:
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
			
			theta += R/(2 * d) * (right_ang_speed - left_ang_speed) * t
			x1 += 1/(2 * R) * (left_ang_speed + right_ang_speed) * np.cos(theta) * t
			y1 += 1/(2 * R) * (left_ang_speed + right_ang_speed) * np.sin(theta) * t


		X[j*N + i] = x1/instant
		Y[j*N + i] = y1/instant

points = np.array([(xi, yi) for xi, yi in zip(X, Y)])
hull = ConvexHull(points)
valid_ids = np.append(hull.vertices, hull.vertices[0])

plt.scatter(X, Y)
plt.plot(points[valid_ids, 0], points[valid_ids, 1], color="C1")
plt.axis("equal")
plt.show()