import pygame
from pygame.locals import *
import util as ut
 
import time
import numpy as np

def draw_point(window, p, delay, color= (255, 0, 0), size = 10):
	pygame.draw.circle(window, color, p, size)
	pygame.display.update()
	pygame.time.delay(delay)

def draw_robot(window, robot_pos):
		"""Draw the robot as a circle

		Args:
			window (surface): Surface on which to draw
		"""
		color = (218, 232, 252)
		border_color = (108, 142, 191)
		# Calculate the vertices of an equilateral triangle
		xm = np.cos(2 * np.pi / 3)
		points = [
			(robot_pos[0] + 20, robot_pos[1]),
			(robot_pos[0] + 20 * xm, robot_pos[1] + 20 * np.sin(2 * np.pi / 3)),
			(robot_pos[0] + 20 * xm, robot_pos[1] + 20 * np.sin(4 * np.pi / 3))
		]
		pygame.draw.polygon(window, color, points)
		pygame.draw.polygon(window, border_color, points, 3)

def draw_waypoint(window, waypoint_pos):
		color = (225, 213, 231)
		border_color = (150, 115, 166)
		pygame.draw.circle(window, color, waypoint_pos, 20)
		pygame.draw.circle(window, border_color, waypoint_pos, 20, 3)

def draw_local_waypoint(window, local_waypoint_pos):
		color = (200, 0, 255)
		pygame.draw.circle(window, color, local_waypoint_pos, 8)


def draw_map(window, map):
	color = (245, 245, 245)
	border_color = (102, 102, 102)
	for c in map:
		match c[0]:
			case 'line':
				pygame.draw.line(window, color, c[1], c[2], 3)
			case 'circle':
				pygame.draw.circle(window, color, c[1], c[2])
				pygame.draw.circle(window, border_color, c[1], c[2], 3)
			case 'rect':
				rect = (c[1][0], c[1][1], abs(c[1][0] - c[3][0]), abs(c[1][1] - c[3][1]))
				pygame.draw.rect(window, color, rect)
				pygame.draw.rect(window, border_color, rect, 3)


def update_display(window, map, robot_pos, waypoint_pos, safe_local_waypoint, safe_local_waypoints, points):
	"""Update the display

	Args:
		window (surface): Surface on which the scene is drawn
	"""
	
	# Erase the display
	# window.fill((150, 150, 150))
	window.fill((255, 255, 255))

	# Draw the map
	draw_map(window, map)
	# Draw the robot
	# Draw the local waypoints
	# draw_local_waypoint(window, safe_local_waypoint)
	# pygame.draw.line(window, (10, 10, 10), robot_pos, safe_local_waypoint, 2)

	if safe_local_waypoints:
		pygame.draw.line(window, (10, 10, 10), robot_pos, safe_local_waypoints[0], 1)
		for i in range(len(safe_local_waypoints) - 1):
			draw_local_waypoint(window, safe_local_waypoints[i])
			pygame.draw.line(window, (10, 10, 10), safe_local_waypoints[i], safe_local_waypoints[i+1], 1)
	

	if len(points):
			pygame.draw.line(window, (10, 10, 10), robot_pos, points[0], 1)
			for i in range(len(points) - 1):
				pygame.draw.circle(window, (0, 255, 200), points[i], 5)
				pygame.draw.line(window, (10, 10, 10), points[i], points[i+1], 1)

	# Draw the waypoint
	draw_waypoint(window, waypoint_pos)
	# pygame.draw.line(window, (10, 10, 10), safe_local_waypoint, waypoint_pos, 2)

	draw_robot(window, robot_pos)


	# Update scene display
	pygame.display.update()


def need_line_split(map, p1, p2, line, safe_range, window):

	for c in map:
		match c[0]:
			case 'line':
				if not ut.compute_segment_inter(p1, p2, c[1], c[2]) is False:
					return True
			case 'circle':
				if p1 == p2:
					return True
				line = ut.compute_line(p1, p2)
				ortho_point = ut.orthogonal_point(c[1], p1, p2, line)
				if ut.distance(c[1], ortho_point) < c[2] + safe_range:
					if ut.point_in_box(ortho_point, p1, p2):
						return True
			case 'rect':
				lines = [(c[4], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[4])]
				for p3, p4 in lines:
					# if ut.distance(p1, p3) < 1e-5:
					# 	return True
					# if ut.distance(p1, p4) < 1e-5:
					# 	return True
					# if ut.distance(p2, p3) < 1e-5:
					# 	return True
					# if ut.distance(p2, p4) < 1e-5:
					# 	return True
					if not ut.compute_segment_inter(p1, p2, p3, p4) is False:
						return True
					# line = ut.compute_line(p3, p4)
					# d = ut.orthogonal_projection(p1, line)
					# if d < safe_range:
					# 	return True
					# d = ut.orthogonal_projection(p2, line)
					# if d < safe_range:
					# 	return True
					
					# line = ut.compute_line(p1, p2)
					# d = ut.orthogonal_projection(p3, line)
					# if d < safe_range:
					# 	return True
					# d = ut.orthogonal_projection(p4, line)
					# if d < safe_range:
					# 	return True
					
	return False
			
def is_safe_point(map, p, safe_range):
	if not ut.point_in_box(p, (0, 0), (1200, 700)):
		return False
	for c in map:
		match c[0]:
			case 'line':
				line = ut.compute_line(c[1], c[2])
				dist = ut.orthogonal_projection(p, line)
				if dist > safe_range:
					continue
				ortho_point = ut.orthogonal_point(p, c[1], c[2], line)
				if ut.point_in_box(ortho_point, c[1], c[2]):
					return False
				if ut.point_in_circle(ortho_point, c[1], safe_range):
					return False
				if ut.point_in_circle(ortho_point, c[2], safe_range):
					return False

			case 'circle':
				if ut.distance(c[1], p) < c[2] + safe_range:
					return False
			
			case 'rect':
				if ut.point_in_box(p, c[1], c[3]):
					return False
				
				lines = [(c[4], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[4])]
				
				for p1, p2 in lines:
					line = ut.compute_line(p1, p2)
					dist = ut.orthogonal_projection(p, line)
					if dist > safe_range:
						continue
					ortho_point = ut.orthogonal_point(p, p1, p2, line)
					if ut.point_in_box(ortho_point, p1, p2):
						return False
					if ut.point_in_circle(ortho_point, p1, safe_range):
						return False
					if ut.point_in_circle(ortho_point, p2, safe_range):
						return False

	return True

			
	

def split_line(map, map_size, p1, p2, line, safe_range, window, imposed_sign = 0):
	a, b, _ = line	# ax + by = c
	if a:
		ortho_vec = ut.normalize_vec((a, b))
	else:
		ortho_vec = (0, ut.sign(b)) 
	
	mdp_1 = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	mdp_2 = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	match imposed_sign:
		case 1:
			mdp_2 = None
		case -1:
			mdp_1 = None

	cpt = 0
	while True and cpt < 1000:
		cpt += 1
		if mdp_1:
			mdp_1 = (mdp_1[0] + max(5, 0.25 * safe_range) * ortho_vec[0], mdp_1[1] + max(5, 0.25 * safe_range) * ortho_vec[1])
			
			if is_safe_point(map, mdp_1, safe_range):
				return mdp_1

			if not ut.point_in_box(mdp_1, (0, 0), map_size):
				mdp_1 = None
			

		if mdp_2:
			mdp_2 = (mdp_2[0] - max(5, 0.25 * safe_range) * ortho_vec[0], mdp_2[1] - max(5, 0.25 * safe_range) * ortho_vec[1])

			if is_safe_point(map, mdp_2, safe_range):
				return mdp_2

			if not ut.point_in_box(mdp_2, (0, 0), map_size):
				mdp_2 = None
		

		if mdp_1 is None and mdp_2 is None:
			return None
	
	print("No split line possible !")
	return None

def find_next_local_waypoint(map, map_size, robot_pos, waypoint_pos, safe_range, window):
	cpt = 0
	safe_local_waypoint = waypoint_pos
	mdp = waypoint_pos
	while True and cpt < 100:
		cpt += 1
		line = ut.compute_line(robot_pos, safe_local_waypoint)
		if need_line_split(map, robot_pos, safe_local_waypoint, line, safe_range, window):
			mdp = split_line(map, map_size, robot_pos, safe_local_waypoint, line, safe_range, window)
			if mdp:
				safe_local_waypoint = mdp
			else:
				print(f"No path available with this safe range : {safe_range}")
				safe_range -= 1
				if safe_range < 10:
					print(f"No path available !!")
					return
					
		else:
			return mdp
	print(f"No path available with this safe range : {safe_range}")
	return

def find_global_path(map, map_size, robot_pos, waypoint_pos, safe_range, window):
	valid_sub_wp = [robot_pos, waypoint_pos]

	cpt = 0
	while True and cpt < 50:
		cpt += 1

		new_valid_sub_wp = [robot_pos]
		line_splited = False

		for i in range(len(valid_sub_wp) - 1):
			p1 = valid_sub_wp[i]
			# draw_point(window, p1, 100, (0, 0, 230))
			p2 = valid_sub_wp[i+1]


			line = ut.compute_line(p1, p2)
			if need_line_split(map, p1, p2, line, safe_range, window):
				line_splited = True
				# mdp = find_next_local_waypoint(map, map_size, p1, p2, safe_range, window)
				mdp = split_line(map, map_size, p1, p2, line, safe_range, window)

				if mdp:
					new_valid_sub_wp.append(mdp)
				else:
					print(f"No path available with this safe range : {safe_range}")
					return
			new_valid_sub_wp.append(p2)
		

		valid_sub_wp = shorten_path(map, new_valid_sub_wp[:], safe_range, window)
		# for map 1 and 3
		# valid_sub_wp = new_valid_sub_wp[:]
		# update_display(window, map, valid_sub_wp[0], valid_sub_wp[-1], valid_sub_wp[1], valid_sub_wp)

		if not line_splited:
			valid_sub_wp = shorten_path(map, new_valid_sub_wp[:], safe_range, window)
			return valid_sub_wp
	
	return

def shorten_path(map, path, safe_range, window):
	i = 0
	p1 = path[i]
	sh_path = [p1]
	while i < len(path) - 1:
		for n in range(i+1, len(path)):
			p2 = path[n]
			if need_line_split(map, p1, p2, ut.compute_line(p1, p2), safe_range, window):
				sh_path.append(path[n-1])
				break
		i = n
		
		p1 = path[i-1]
	sh_path.append(path[-1])

	return sh_path

def move_robot(robot_pos, safe_local_waypoint):
	goal_line = ut.compute_line(safe_local_waypoint, robot_pos)
	a, b, _ = goal_line
	if a:
		dir_vec = ut.normalize_vec((b, -a))
	else:
		dir_vec = (ut.sign(a), 0) 
	return (robot_pos[0] + 10 * dir_vec[0], robot_pos[1] + 10 * dir_vec[1])

def rd():
	rdmin = -10
	rdmax = 10
	return np.random.randint(rdmin, rdmax)

def main():
	### INITIALISAZE PYGAME
	pygame.init()

	### WINDOW INITIALISATION
	window_size = (1200, 700)
	# Initialize the window
	window = pygame.display.set_mode(window_size)
	window.fill((150, 150, 150))
	# Set the title of the window
	pygame.display.set_caption("Path difference - Map 2")


	### WORLD INITIALISATION
	robot_pos = (100, 350)
	last_static_pos = robot_pos
	waypoint_pos = (1100, 350)

	# robot_pos = (100, 100)
	# last_static_pos = robot_pos
	# waypoint_pos = (100, 450)

	# MAP 1
	map1 = [('circle', (600, 350), 200)]

	# MAP 2
	map2 = [('rect', (400, 150), (800, 150), (800, 550), (400, 550))]

	# MAP 3
	map3 = [('circle', (550, 200), 100),
		   ('circle', (550, 500), 100),
		   ('circle', (650, 350), 100)]
	
	# MAP 4
	map4 = [('rect', (550, 0), (650, 0), (650, 550), (550, 550)),
		   ('rect', (550, 600), (650, 600), (650, 1000), (550, 1000))]
	
	# MAP 5
	map5 = [('rect', (700, 0), (800, 0), (800, 600), (700, 600)),
		   ('rect', (400, 100), (500, 100), (500, 1000), (400, 1000))]
	
	# MAP 6
	map6 = [('rect', (300, 150), (900, 150), (900, 200), (300, 200)),
		 	('rect', (300, 500), (900, 500), (900, 550), (300, 550)),
			('rect', (850, 150), (900, 150), (900, 550), (850, 550))]
	
	# MAP 7
	map7 = [('rect', (300, 150), (900, 150), (900, 200), (300, 200)),
			('rect', (850, 150), (900, 150), (900, 1000), (850, 1000))]

	# MAP 8
	map8 = [('rect', (850, 0), (900, 0), (900, 200), (850, 200)),
	 		('rect', (0, 300), (900, 300), (900, 350), (0, 350)),
	 		('rect', (350, 350), (400, 350), (400, 600), (350, 600)),
			('rect', (600, 500), (1200, 500), (1200, 550), (600, 550))]

	map = map5


	# State of the simulation
	running = True
	next_it = False
	text_shown = False

	# Simulation variables 
	safe_local_waypoint = waypoint_pos
	total_dist = 0
	safe_range = 20
	static_pos_counter = 0
	look_for_path_through_known_map = False



	i = 0
	while running:
		# Handle events		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
			# Handle if the key was pressed once
			if event.type == KEYDOWN:
				if event.key == K_SPACE:
					next_it = True
				if event.key == K_ESCAPE: 
					running = False
		
		if not next_it:
			if not text_shown:
				font = pygame.font.Font('freesansbold.ttf', 16)
				text = font.render('Next iteration ? (Press space)', True, (255, 255, 255))

				textRect = text.get_rect()
		
				textRect.center = (window_size[0]*0.9, 10)
				# window.blit(text, textRect)
				# draw_map(window, map)
				pygame.display.update()

				text_shown = True
				
			continue


		### SIMULATION
		points = []
		next_point = robot_pos
		while ut.distance(next_point, waypoint_pos) > 1e-5:
			next_point = find_next_local_waypoint(map, window_size, next_point, waypoint_pos, 0, window)
			points.append(next_point)

		if points:
			total_path_length = np.sum([ut.distance(points[i], points[i+1]) for i in range(len(points[1::]))])
			print(total_path_length + ut.distance(robot_pos, points[0]))	

		i += 1

		if ut.point_in_circle(robot_pos, waypoint_pos, 10):
			continue
		
		###   GLOBAL PATH   ###
		start_time_cpt = time.perf_counter()
		
		safe_local_waypoints = find_global_path(map, window_size, robot_pos, waypoint_pos, safe_range, window)

		end_time_cpt = time.perf_counter()

		print(f"Global path enlapse time : {end_time_cpt-start_time_cpt:.3e}s")
		print(f"Global path FPS : {1/(end_time_cpt-start_time_cpt):.1f} FPS")

		if safe_local_waypoints is not None:
			total_path_length = np.sum([ut.distance(safe_local_waypoints[i], safe_local_waypoints[i+1]) for i in range(len(safe_local_waypoints[1::]))])
			print(total_path_length)


		# DRAW THE SCENE
		# update_display(window, map, robot_pos, waypoint_pos, safe_local_waypoint, None)
		update_display(window, map, robot_pos, waypoint_pos, None, safe_local_waypoints, points)
		text_shown = False
		next_it = False

	### DEINITIALIZE PYGAME
	pygame.quit()


if __name__ == '__main__':
	main()