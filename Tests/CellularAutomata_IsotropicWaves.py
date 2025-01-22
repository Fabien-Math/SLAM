# Based on the WAVES in ISOTROPIC TOTALISTIC CELLULAR AUTOMATA: APPLICATION to REAL-TIME ROBOT NAVIGATION Article in Advances in Complex Systems · December 2016

import numpy as np
import matplotlib.pyplot as plt
import tqdm

def norm2(x):
	return np.sqrt(x[0]**2 + x[1]**2)

def distance(p, q):
	return np.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

def moore_neighbors(i, j, n, m):
	# Faster than initilizing Mij to [0]*8
	Mij = []
	for k in [-1, 0, 1]:
		for l in [-1, 0, 1]:
			if k != 0 or l != 0:
				if 0 <= i + k < n and 0 <= j + l < m:
					Mij.append((i + k, j + l))
	
	return Mij

def calc_rt(i, j, k, l, a_lattice, z_lattice):
	if a_lattice[i, j] or not a_lattice[k, l]:
		return (0, 0)
	
	return z_lattice[k, l] + [1, abs(k - i)*abs(l - j)]
	
def ijstar(i, j, Wt, a_lattice, z_lattice):
	if not len(Wt):
		return (i, j)

	d_min = 1e9
	for k, l in Wt:
		d = norm2(calc_rt(i, j, k, l, a_lattice, z_lattice))
		if d < d_min:
			d_min = d
			istar = k
			jstar = l

	return (istar, jstar)


def update_lattice(t, a_lattice, z_lattice):
	new_a_lattice = a_lattice.copy()
	new_z_lattice = z_lattice.copy()

	for i in range(n):
		for j in range(m):
			obstacle = False
			for obs in B:
				if len(obs) == 4: 	# Rectangular obstacle
					if obs[0] < i < obs[1]:
						if obs[2] < j < obs[3]:
							obstacle = True
							break
				if len(obs) == 2:	# Circular obstacle
					centre, radius = obs
					if distance((i,j), centre) < radius:
						obstacle = True
						break
			
			if obstacle:
				continue

			if a_lattice[i, j]:
				continue

			Mij = moore_neighbors(i, j, n, m)

			sum = 0
			for k, l in Mij:
				sum += a_lattice[k, l]
			
			if not sum:
				continue


			Wtij = [(k, l) for (k, l) in Mij if t < a_lattice[k, l] + norm2(calc_rt(i, j, k, l, a_lattice, z_lattice)) <= t + 1]
			# print([a_lattice[k, l] + norm2(calc_rt(i, j, k, l, a_lattice, z_lattice)) for (k, l) in Mij])


			istar, jstar = ijstar(i, j, Wtij, a_lattice, z_lattice)

			if (i, j) in Gamma and sum:
				new_a_lattice[i, j] = t + 1
			else:
				new_a_lattice[i, j] = a_lattice[istar, jstar]


			if not ((a_lattice[i, j] and (i, j) not in Gamma) or not len(Wtij)):
				new_z_lattice[i, j] = calc_rt(i, j, istar, jstar, a_lattice, z_lattice)
	
	a_lattice = new_a_lattice.copy()
	z_lattice = new_z_lattice.copy()

	t += 1

	return t, a_lattice, z_lattice

scale = 1
n, m = int(1200 / scale), int(700 / scale)

a_lattice = np.zeros((n, m))
z_lattice = np.zeros((n, m, 2))
Gamma = []
B = [((600/scale, 350/scale), 200/scale)]

a_lattice[int(100 / scale), int(350 / scale)] = 1

t = 1
for i in tqdm(range(2000)):
	t, a_lattice, z_lattice = update_lattice(t, a_lattice, z_lattice)
	if a_lattice[int(1100 / scale), int(350 / scale)]:
		print("Time to reach the waypoint :", t)
		break
	# plt.figure(1)
	# plt.clf()
	# plt.imshow(a_lattice, cmap='gray')
	# plt.pause(0.01)




plt.show()