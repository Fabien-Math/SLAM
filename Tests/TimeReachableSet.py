import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import itertools as it

def make_possibilities(N):

	left_right = []
	for p1 in poss:
		for p2 in poss:
			for p3 in poss:
				left_right.append([p1, p2, p3])
	
	return left_right
	
max_left_ang_speed = 1
max_right_ang_speed = 1
R = 0.04
d = 0.15
instants = [1, 2, 3 ]


poss = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]


N = 1000
X, Y = [], []
for j, instant in enumerate(instants):

	left_right = np.array([p for p in it.combinations(poss, instant)])
	print(left_right)
	print(len(left_right))

	time = np.linspace(0, 1, instant)

	for lr_speed in left_right:
		right_ang_speed, left_ang_speed = lr_speed[0]

		theta = np.zeros(instant)
		for i, t in enumerate(time[1::]):
			theta_i = R/(2 * d) * (right_ang_speed - left_ang_speed)
			theta[i+1] = theta[i] + theta_i
		x1 = np.sum(1/(2 * R) * (left_ang_speed + right_ang_speed) * np.cos(theta) * time) / instant
		y1 = np.sum(1/(2 * R) * (left_ang_speed + right_ang_speed) * np.sin(theta) * time) / instant

		X.append(x1)
		Y.append(y1)

points = np.array([(xi, yi) for xi, yi in zip(X, Y)])
hull = ConvexHull(points)
valid_ids = np.append(hull.vertices, hull.vertices[0])

plt.scatter(X, Y)
plt.plot(points[valid_ids, 0], points[valid_ids, 1], color="C1")
plt.axis("equal")
plt.show()