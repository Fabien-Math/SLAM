# Based on the WAVES in ISOTROPIC TOTALISTIC CELLULAR AUTOMATA: APPLICATION to REAL-TIME ROBOT NAVIGATION Article in Advances in Complex Systems · December 2016

import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from matplotlib.animation import ArtistAnimation, PillowWriter
plt.rcParams.update({'font.size': 18})


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

def in_obstacle(i, j, B):
	for obs in B:
		if len(obs) == 4: 	# Rectangular obstacle
			if obs[0] <= i <= obs[1]:
				if obs[2] <= j <= obs[3]:
					return True
		if len(obs) == 2:	# Circular obstacle
			centre, radius = obs
			if distance(centre, (i, j)) <= radius:
				return True
	
	return False

def compute_angle(p1, p2):
	ang = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
	return ang + (2 * np.pi)*(ang < 0)


def order_point(vertexes):
	middle_point = (np.sum([v[0] for v in vertexes])/len(vertexes), np.sum([v[1] for v in vertexes])/len(vertexes))

	angles = [compute_angle(middle_point, v) for v in vertexes]

	ordered_vertexes = [v for _, v in sorted(zip(angles, vertexes))]

	return ordered_vertexes


def build_gamma(B, n, m, lambda_ch):
	potential_gamma = []
	for id_obs, obstacle in enumerate(B):
		potential_gamma.append([])
		match len(obstacle):
			case 4:
				for i in range(obstacle[0], obstacle[1]+1):
					for j in range(obstacle[2], obstacle[3]+1):
						Mij = moore_neighbors(i, j, n, m)
		
						cpt = 0
						for k, l in Mij:
							if in_obstacle(k, l, B):
								cpt += 1

						if cpt < 5:
							potential_gamma[id_obs].append((i, j))
			case 2:
				c, r = obstacle
				for i in range(c[0] - r, c[0] + r + 1):
					for j in range(c[1] - r, c[1] + r + 1):
						if not in_obstacle(i, j, B):
							continue

						Mij = moore_neighbors(i, j, n, m)
		
						cpt = 0
						for k, l in Mij:
							if in_obstacle(k, l, B):
								cpt += 1

						if cpt < 5:
							potential_gamma[id_obs].append((i, j))

			case _:
				break

	Gamma = []
	for p_gamma in potential_gamma:
		ordered_p_gamma = order_point(p_gamma)
		len_opg = len(ordered_p_gamma)

		for i in range(len_opg):
			iqp, jqp = ordered_p_gamma[(i+1)%len_opg]
			iq, jq = ordered_p_gamma[i]
			iqm, jqm = ordered_p_gamma[i-1]
			kq, lq = iqp - iq, jqp - jq
			kqm, lqm = iq - iqm, jq - jqm

			if lambda_ch * (kqm * lq - kq * lqm) > max(0, kqm*kq + lqm*lq):
				Gamma.append(ordered_p_gamma[i])

	return Gamma



def update_lattice(t, n, m, a_lattice, z_lattice, B, Gamma):
	new_a_lattice = a_lattice.copy()
	new_z_lattice = z_lattice.copy()

	for i in range(n):
		for j in range(m):
			in_obst = in_obstacle(i, j, B)
			if in_obst:
				if (i, j) not in Gamma:
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


			istar, jstar = ijstar(i, j, Wtij, a_lattice, z_lattice)

			if in_obst and sum:
				if (i,j) in Gamma:
					new_a_lattice[i, j] = t + 1
				else:
					new_a_lattice[i, j] = a_lattice[istar, jstar]
			else:
				new_a_lattice[i, j] = a_lattice[istar, jstar]

			if not(in_obst or a_lattice[i, j]) and len(Wtij):
				new_z_lattice[i, j] = calc_rt(i, j, istar, jstar, a_lattice, z_lattice)
	
	if np.sum(a_lattice) == np.sum(new_a_lattice):
		return None, a_lattice, z_lattice

	a_lattice = new_a_lattice.copy()
	z_lattice = new_z_lattice.copy()

	t += 1

	return t, a_lattice, z_lattice


####################################################################################################
# TEST CASES
####################################################################################################

def convert_obstacles(obstacles, scale):
	B = [None]*len(obstacles)
	for i, obs in enumerate(obstacles):
		match obs[0]:
			case 'circle':
				B[i] = ((int(obs[1][0]*scale), int(obs[1][1]*scale)), int(obs[2]*scale))
			case 'rect':
				B[i] = (int(obs[1][0]*scale), int(obs[3][0]*scale), int(obs[1][1]*scale), int(obs[3][1]*scale))
	return B


def compute_and_save_shortest_path(source_pos, wp_pos, map_size, scale, obstacles, granularity, name):
	n, m = int(map_size[0]*scale), int(map_size[1]*scale)
	a_lattice = np.zeros((n, m))
	z_lattice = np.zeros((n, m, 2))

	B = convert_obstacles(obstacles, scale)
	Gamma = build_gamma(B, n, m, granularity)

	a_lattice[int(source_pos[0]*scale), int(source_pos[1]*scale)] = 1

	t = 1

	frames = []
	
	for i in tqdm(range(2*int(np.sqrt(n**2 + m**2)))):
		t, a_lattice, z_lattice = update_lattice(t, n, m, a_lattice, z_lattice, B, Gamma)

		frames.append([display_grid(a_lattice, scale, wp_pos, (t-2)/scale, obstacles, wp_color=True)])
	
		if a_lattice[int(wp_pos[0]*scale), int(wp_pos[1]*scale)] > 0:
			break
		if t is None:
			break
		
	
	fig = plt.figure("Test case", figsize=(19.2, 10.8), layout="constrained")
	ani = ArtistAnimation(fig, frames, interval=40, blit=True)
	writer = PillowWriter(fps=25)

	ani.save('Images/' + name + '.gif', writer=writer)
	print(f"Saved as Images/{name}.gif")

	if a_lattice[int(wp_pos[0]*scale), int(wp_pos[1]*scale)] > 0:
		return (1/scale)*(t-2), a_lattice
	
	if t is None:
			return None, a_lattice


def compute_shortest_path(source_pos, wp_pos, map_size, scale, obstacles, granularity):
	n, m = int(map_size[0]*scale), int(map_size[1]*scale)
	a_lattice = np.zeros((n, m))
	z_lattice = np.zeros((n, m, 2))

	B = convert_obstacles(obstacles, scale)
	Gamma = build_gamma(B, n, m, granularity)

	a_lattice[int(source_pos[0]*scale), int(source_pos[1]*scale)] = 1

	t = 1
	for i in tqdm(range(2*int(np.sqrt(n**2 + m**2)))):
		t, a_lattice, z_lattice = update_lattice(t, n, m, a_lattice, z_lattice, B, Gamma)

		if a_lattice[int(wp_pos[0]*scale), int(wp_pos[1]*scale)] > 0:
			return (1/scale)*(t-2), a_lattice
		
		if t is None:
			return None, a_lattice
		


def display_grid(a_lattice, scale, wp_pos, time, obstacles, wp_color = True, obst_color = True, one_color = True, sec_colors = False):
	plt.figure("Test case", figsize=(19.2, 10.8), layout="constrained")

	B = convert_obstacles(obstacles, scale)
	a_lattice_plot = a_lattice.copy()

	# Only one color for the wave
	if one_color:
		mask = a_lattice > 0
		a_lattice_plot[mask] = 2
	
	# Display obstacle in a different from the others
	if obst_color:
		for i in range(a_lattice.shape[0]):
			for j in range(a_lattice.shape[1]):
				if in_obstacle(i, j, B):
					a_lattice_plot[i, j] = -1

	# Colors from secondary sources
	if sec_colors:
		for i, v in enumerate(np.unique(a_lattice)):
			mask = a_lattice == v
			a_lattice_plot[mask] = i

	
	# Waypoint color
	if wp_color:
		a_lattice_plot[int(wp_pos[0]*scale), int(wp_pos[1]*scale)] = 1

	# Update image
	plt.title(f"Isotropic wave, minimal distance to reach the target : {time} um")
	plt.axis('off')
	return plt.imshow(np.transpose(a_lattice_plot), cmap='Blues', interpolation='nearest')
	# plt.title(f"Isotropic wave, minimal distance to reach the target : {time} um" + r"$\pm$" + f"{2/scale:2g} um")