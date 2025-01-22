# Based on the WAVES in ISOTROPIC TOTALISTIC CELLULAR AUTOMATA: APPLICATION to REAL-TIME ROBOT NAVIGATION Article in Advances in Complex Systems · December 2016

import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

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

def in_obstacle(i, j):
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
							if in_obstacle(k, l):
								cpt += 1

						if cpt < 5:
							# plt.scatter(i, j)
							potential_gamma[id_obs].append((i, j))
			case 2:
				c, r = obstacle
				for i in range(c[0] - r, c[0] + r + 1):
					for j in range(c[1] - r, c[1] + r + 1):
						if not in_obstacle(i, j):
							continue

						Mij = moore_neighbors(i, j, n, m)
		
						cpt = 0
						for k, l in Mij:
							if in_obstacle(k, l):
								cpt += 1

						if cpt < 5:
							# plt.scatter(i, j, color='red')
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




def update_lattice(t, a_lattice, z_lattice, Gamma):
	new_a_lattice = a_lattice.copy()
	new_z_lattice = z_lattice.copy()

	for i in range(n):
		for j in range(m):
			if (i, j) not in Gamma:
				if in_obstacle(i, j):
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

			# if not (a_lattice[i, j] and not in_obstacle(i, j) or len(Wtij) == 0):
			# print( (not(in_obstacle(i, j) or a_lattice[i, j]) or len(Wtij)) == (not ((a_lattice[i, j] and (i, j) not in Gamma) or not len(Wtij))))
			if not(in_obstacle(i, j) or a_lattice[i, j]) and len(Wtij):
			# if not ((a_lattice[i, j] and (i, j) not in Gamma) or not len(Wtij)):
				new_z_lattice[i, j] = calc_rt(i, j, istar, jstar, a_lattice, z_lattice)
	
	a_lattice = new_a_lattice.copy()
	z_lattice = new_z_lattice.copy()

	t += 1

	return t, a_lattice, z_lattice


# n, m = 101, 101

# a_lattice = np.zeros((n, m))
# z_lattice = np.zeros((n, m, 2))

# # (xmin, xmax, ymin, ymax)
# B = [(50, 100, 40, 45), ((20, 50), 10)]
# # B = [(50, 100, 40, 45)]
# lambda_ch = 5
# Gamma = build_gamma(B, n, m, lambda_ch)
# a_lattice[50, 75] = 1

# t = 1
# for i in tqdm(range(int(100))):
# 	t, a_lattice, z_lattice = update_lattice(t, a_lattice, z_lattice, Gamma)
# 	# for i,j in Gamma:
# 	# 	if a_lattice[i, j]:
# 	# 		print('Gamma', i, j)
# 	# 		plt.pause(1)
# 	plt.figure(1)
# 	plt.clf()
# 	modif_a_lattice = a_lattice.copy()

# 	for i in range(n):
# 		for j in range(m):
# 			if in_obstacle(i, j):
# 				modif_a_lattice[i, j] = -1
# 	for i,j in Gamma:
# 		modif_a_lattice[i, j] = -0.5
# 	plt.imshow(modif_a_lattice, cmap='rainbow')
# 	plt.pause(0.01)



####  SHORTEST PATH CASE   ##### 
scale = 2
n, m = int(700 / scale), int(1200 / scale)

a_lattice = np.zeros((n, m))
z_lattice = np.zeros((n, m, 2))
# B = []
# B = [((int(350/scale), int(600/scale)), int(200/scale))]
B = [(int(150 / scale), int(550 / scale), int(400 / scale), int(800 / scale))]
lambda_ch = 5
Gamma = build_gamma(B, n, m, lambda_ch)
# print(Gamma)
# exit()

a_lattice[int(350 / scale), int(100 / scale)] = 1

t = 1
# plt.figure(1, layout='constrained')
# for i in tqdm(range(int(1200/scale))):
# 	t, a_lattice, z_lattice = update_lattice(t, a_lattice, z_lattice, Gamma)
# 	if a_lattice[int(350 / scale), int(1100 / scale)]:
# 		print("Time to reach the waypoint :", (t-1) * scale)
# 		break
	
# 	mask = a_lattice > 0
# 	a_lattice_plot = a_lattice.copy()
# 	a_lattice_plot[mask] = 1

	# # WITH COLOR FROM SECONDARY SOURCES
	# a_lattice_plot = a_lattice.copy()
	# for i, v in enumerate(np.unique(a_lattice)):
	# 	mask = a_lattice == v
	# 	a_lattice_plot[mask] = i

# 	for i in range(n):
# 		for j in range(m):
# 			if in_obstacle(i, j):
# 				a_lattice_plot[i, j] = -1
# 	plt.clf()
# 	plt.imshow(a_lattice_plot, cmap='Blues')
# 	plt.pause(0.01)




# # plt.imshow(a_lattice_plot, cmap='gray')
# # plt.show()

from matplotlib.animation import FuncAnimation, PillowWriter

# # Étape 3 : Fonction d'animation
# def update(frame, t, a_lattice, z_lattice, Gamma, scale):
# 	t, a_lattice, z_lattice = update_lattice(t, a_lattice, z_lattice, Gamma)
# 	if a_lattice[int(350 / scale), int(1100 / scale)]:
# 		print("Time to reach the waypoint :", (t-1) * scale)
# 		return
	
# 	mask = a_lattice > 0
# 	a_lattice_plot = a_lattice.copy()
# 	a_lattice_plot[mask] = 1

# 	# WITH COLOR FROM SECONDARY SOURCES
# 	a_lattice_plot = a_lattice.copy()
# 	for i, v in enumerate(np.unique(a_lattice)):
# 		mask = a_lattice == v
# 		a_lattice_plot[mask] = i

# 	for i in range(n):
# 		for j in range(m):
# 			if in_obstacle(i, j):
# 				a_lattice_plot[i, j] = -1


# 	plt.clf()
# 	plt.imshow(a_lattice_plot, cmap='Blues')
# 	return

# fig = plt.figure("Animation Isotropic Wave", layout='constrained')

# num_frames = 100
# interval = 40  # Intervalle de 50ms entre les frames
# ani = FuncAnimation(fig, update, frames=num_frames, interval=interval, fargs=(t, a_lattice, z_lattice, Gamma, scale))

# # Étape 5 : Enregistrement de l'animation au format GIF
# filename = "Tests/animation.gif"
# writer = PillowWriter(fps=25)  # Définir les images par seconde
# ani.save(filename, writer=writer)
# print(f"Animation sauvegardée sous le nom {filename}")


# Étape 3 : Fonction d'animation
def update(frame):
	global t, a_lattice, z_lattice, Gamma, scale
	print(t, '/', int(1200 / scale), end='\r')
	# Met à jour le réseau en fonction de votre logique
	t, a_lattice, z_lattice = update_lattice(t, a_lattice, z_lattice, Gamma)
	
	# Vérifie une condition pour afficher le temps de parcours
	if a_lattice[int(350 / scale), int(1100 / scale)]:
		print("Time to reach the waypoint :", (t - 1) * scale)
		return
	
	# Crée un masque pour gérer les données
	mask = a_lattice > 0
	a_lattice_plot = a_lattice.copy()
	a_lattice_plot[mask] = 1

	# Applique des couleurs à partir des sources secondaires
	# a_lattice_plot = a_lattice.copy()
	# for i, v in enumerate(np.unique(a_lattice)):
	# 	mask = a_lattice == v
	# 	a_lattice_plot[mask] = i

	# Marque les obstacles
	for i in range(a_lattice.shape[0]):
		for j in range(a_lattice.shape[1]):
			if in_obstacle(i, j):
				a_lattice_plot[i, j] = -1
		
	a_lattice_plot[int(350 / scale), int(1100 / scale)] = 0.5

	# Met à jour l'image de la matrice
	plt.clf()  # Efface la figure pour actualiser correctement
	plt.imshow(a_lattice_plot, cmap='Blues', interpolation='nearest')
	# plt.suptitle(f"")
	plt.title(f"Isotropic wave, minimal distance to reach the target : {(t-1) * scale} um")
	return  # Pas besoin de retourner un objet avec plt.imshow

# Étape 4 : Initialisation de la figure
fig = plt.figure("Animation Isotropic Wave", layout='constrained')

# Paramètres d'animation
num_frames = int(1200/scale)
interval = 40  # Intervalle en millisecondes entre les frames

# Crée l'animation
ani = FuncAnimation(
	fig, update, frames=num_frames, interval=interval, blit=False
)

# Étape 5 : Enregistrement de l'animation au format GIF
filename = "Tests/isotropic_wave_square_obst_anim.gif"
writer = PillowWriter(fps=25)  # Définir les images par seconde
ani.save(filename, writer=writer)
print(f"Animation sauvegardée sous le nom {filename}")


