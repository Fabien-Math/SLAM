import numpy as np
import matplotlib.pyplot as plt
import time

n, m = 9*6+1, 16*6+1
table = np.zeros((n, m))

init_square = [[0,0,0,0,0],
			   [0,0,1,0,0],
			   [0,1,1,0,0],
			   [0,0,1,1,0],
			   [0,0,0,0,0]]
p,q = np.shape(init_square)

table[n//2 - p//2: n//2 - p//2 + p, m//2 - q//2: m//2 - q//2 + q] = init_square

def norm(x, y):
	return np.sqrt(x**2 + y**2)

def normalize(x, y):
	n = norm(x, y)
	return (x / n, y / n)

def update_table(table : np.array):
	n, m = np.shape(table)
	
	new_table = np.copy(table)
	
	for i in range(1, n-1):
		for j in range(1, m-1):
			# if 55 < i < 60:
			# 	if 20 < j < 80:
			# 		continue
			
			# if new_table[i, j]:
			# 	continue
			# Beginning of the rule
			# next_cell = 0
			# diag_cell = 0
			# #top_left = False
			# # Check up row
			# directions = ((-1, 0), (0, -1), (1, 0), (0, 1))
			# for diri, dirj in directions:
			# 	if table[i + diri, j + dirj]:
			# 		next_cell += 1
			# 		# num_active_cells += norm(diri, dirj)

			# diag_dirs =  ((-1, -1), (-1, 1), (1, -1), (1, 1))
			# for diri, dirj in diag_dirs:
			# 	if table[i + diri, j + dirj]:

			# 		diag_cell += norm(diri, dirj)
			
			# new_table[i, j] += diag_cell + next_cell

			# num_active_cells = np.sum(table[i-1:i+2, j-1:j+2])
			# print(table[i-1:i+2, j-1:j+2], num_active_cells)


# 0.012

			num_active_cells = 0
			if table[i-1, j-1]:
				num_active_cells += 1
			if table[i-1, j]:
				num_active_cells += 1
			if table[i-1, j+1]:
				num_active_cells += 1
			if table[i, j-1]:
				num_active_cells += 1
			if table[i, j+1]:
				num_active_cells += 1
			if table[i+1, j-1]:
				num_active_cells += 1
			if table[i+1, j]:
				num_active_cells += 1
			if table[i+1, j+1]:
				num_active_cells += 1

			
			"""Game of life rules"""
			if table[i, j] and (num_active_cells == 3 or num_active_cells == 2):
				new_table[i, j] = 1
			elif num_active_cells == 3 and not table[i, j]:
				new_table[i, j] = 1 
			elif num_active_cells < 2 or num_active_cells > 3:
				new_table[i, j] = 0
			
			"""Own rules"""
			# if table[i, j] and (num_active_cells == 2 or num_active_cells == 3):
			#     new_table[i, j] = 1
			# elif num_active_cells == 2 and not table[i, j]:
			#     new_table[i, j] = 1 
			# elif num_active_cells < 2 or num_active_cells > 3:
			#     new_table[i, j] = 0

	# mask = new_table != 0
	# new_table[mask] = 1 #new_table[mask].astype(int)
	return new_table


times = []
plt.figure(0)
plt.imshow(init_square, cmap='gray_r')
plt.show()

n_cells = np.sum(table)
cpt = 0
plt.figure("Conway's Game of life", layout="constrained", figsize=(19.2, 10.8))
for i in range(263):
	old_n_cells = n_cells
	# a = time.perf_counter()
	table = update_table(table)
	n_cells = np.sum(table)
	# if n_cells == old_n_cells:
	# 	cpt += 1
	# 	if cpt == 5:
	# 		break
	# else:
	# 	cpt = 0
	# times.append(time.perf_counter() - a)

	plt.clf()
	plt.suptitle("Conway's Game of life")
	plt.title(f"Generation {i} with {n_cells:g} activated", fontsize=18)
	plt.imshow(table, cmap='gray')
	plt.pause(0.01)
	
	
# print(np.mean(times))
# plt.imsave("Images/maze2.png", table, cmap='gray')

