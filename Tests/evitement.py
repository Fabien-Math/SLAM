import pygame
from pygame.locals import *
import util as ut
 
import time
import numpy as np

def draw_robot(window, robot_pos):
		"""Draw the robot as a circle

		Args:
			window (surface): Surface on which to draw
		"""
		color = (255, 0, 100)
		pygame.draw.circle(window, color, robot_pos, 10)

def draw_waypoint(window, waypoint_pos):
		color = (0, 255, 100)
		pygame.draw.circle(window, color, waypoint_pos, 10)

def draw_local_waypoint(window, local_waypoint_pos):
		color = (100, 255, 0)
		pygame.draw.circle(window, color, local_waypoint_pos, 10)


def draw_map(window, map):
	for c in map:
		pygame.draw.circle(window, (0, 0, 0), c[0], c[1], 3)

def update_display(window, map, robot_pos, waypoint_pos, safe_local_waypoint, safe_local_waypoints):
	"""Update the display

	Args:
		window (surface): Surface on which the scene is drawn
	"""
	
	# Erase the display
	window.fill((150, 150, 150))

	# Draw the map
	draw_map(window, map)
	# Draw the robot
	draw_robot(window, robot_pos)
	# Draw the local waypoints
	draw_local_waypoint(window, safe_local_waypoint)
	pygame.draw.line(window, (10, 10, 10), robot_pos, safe_local_waypoint, 2)

	if safe_local_waypoints:
		pygame.draw.line(window, (10, 10, 10), robot_pos, safe_local_waypoints[0], 2)
		for i in range(len(safe_local_waypoints) - 1):
			draw_local_waypoint(window, safe_local_waypoints[i])
			pygame.draw.line(window, (10, 10, 10), safe_local_waypoints[i], safe_local_waypoints[i+1], 2)
	# Draw the waypoint
	draw_waypoint(window, waypoint_pos)
	pygame.draw.line(window, (10, 10, 10), safe_local_waypoint, waypoint_pos, 2)


	# Update scene display
	pygame.display.update()

def need_line_split(map, p1, p2, line, safe_range):
	for c in map:
		ortho_point = ut.orthogonal_point(c[0], p1, p2, line)
		if ut.distance_tuple(c[0], ortho_point) < c[1] + safe_range:
			if ut.point_in_box_tuple(ortho_point, p1, p2):
				return True
	
	return False

def split_line(map, map_size, p1, p2, line, safe_range, imposed_sign = 0):
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
	while True and cpt < 100:
		cpt += 1
		if mdp_1:
			mdp_1 = (mdp_1[0] + 0.25 * safe_range * ortho_vec[0], mdp_1[1] + 0.25 * safe_range * ortho_vec[1])

			safe = True
			for c in map:
				if ut.distance_tuple(c[0], mdp_1) < c[1] + safe_range:
					safe = False
					break
			if safe:
				return mdp_1

			if not ut.point_in_box_tuple(mdp_1, (0, 0), map_size):
				mdp_1 = None
			

		if mdp_2:
			mdp_2 = (mdp_2[0] - 0.25 * safe_range * ortho_vec[0], mdp_2[1] - 0.25 * safe_range * ortho_vec[1])

			safe = True
			for c in map:
				if ut.distance_tuple(c[0], mdp_2) < c[1] + safe_range:
					safe = False
					break
			if safe:
				return mdp_2

			if not ut.point_in_box_tuple(mdp_2, (0, 0), map_size):
				mdp_2 = None
		

		if mdp_1 is None and mdp_2 is None:
			return None
	
	return None

def find_next_local_waypoint(map, map_size, robot_pos, waypoint_pos, safe_range):
	cpt = 0
	safe_local_waypoint = waypoint_pos
	while True and cpt < 1000:
		cpt += 1
		line = ut.compute_line_tuple(robot_pos, safe_local_waypoint)
		if need_line_split(map, robot_pos, safe_local_waypoint, line, safe_range):
			mdp = split_line(map, map_size, robot_pos, safe_local_waypoint, line, safe_range)
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
			
	return

def find_global_path(map, map_size, robot_pos, waypoint_pos, safe_range, window = None):
	valid_sub_wp = [robot_pos, waypoint_pos]

	cpt = 0
	while True and cpt < 100:
		cpt += 1

		new_valid_sub_wp = [robot_pos]
		line_splited = False

		for i in range(len(valid_sub_wp) - 1):
			p1 = valid_sub_wp[i]
			p2 = valid_sub_wp[i+1]
			line = ut.compute_line_tuple(p1, p2)
			if need_line_split(map, p1, p2, line, safe_range):
				line_splited = True
				mdp = find_next_local_waypoint(map, map_size, p1, p2, safe_range)
				if mdp:
					new_valid_sub_wp.append(mdp)
				else:
					print(f"No path available with this safe range : {safe_range}")
					return
			new_valid_sub_wp.append(p2)
				
		valid_sub_wp = new_valid_sub_wp[:]

		if not line_splited:
			return valid_sub_wp
	
	return

def move_robot(robot_pos, safe_local_waypoint):
	goal_line = ut.compute_line_tuple(safe_local_waypoint, robot_pos)
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
	# Set the title of the window
	pygame.display.set_caption("Test evitement")


	### WORLD INITIALISATION
	robot_pos = (200, 350)
	last_static_pos = robot_pos
	waypoint_pos = (1000, 350)
	# map = [((600, 250), 200), ((800, 450), 200)]*10
	map = [((400, 250), 100), ((800, 450), 100), ((700, 200), 100), ((800, 350), 100), ((800, 700), 100)]

	# State of the simulation
	running = True
	next_it = False
	text_shown = False

	# Simulation variables 
	safe_local_waypoint = waypoint_pos
	total_dist = 0
	safe_range = 25
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
				window.blit(text, textRect)
				pygame.display.flip()

				text_shown = True
			continue


		### SIMULATION
		i += 1

		map = [((400 + 100, 250), 100), ((800, 450), 100), ((700, 200), 100), ((800, 350), 100), ((800, 600), 100)]
		# map = [((400 + 100, 250 + rd()), 100), ((800 + rd(), 450 + rd()), 100), ((700 + rd(), 200 + rd()), 100), ((800 + rd(), 350 + rd()), 100), ((800 + rd(), 700 + rd()), 100)]
		# map1 = [((400 + rd(), 250 + rd()), 100), ((800 + rd(), 450 + rd()), 100), ((700 + rd(), 200 + rd()), 100), ((800 + rd(), 350 + rd()), 100), ((800 + rd(), 700 + rd()), 100)]
		# map2 = [((400 + rd(), 250 + rd()), 100), ((800 + rd(), 450 + rd()), 100), ((700 + rd(), 200 + rd()), 100), ((800 + rd(), 350 + rd()), 100), ((800 + rd(), 700 + rd()), 100)]
		# map3 = [((400 + rd(), 250 + rd()), 100), ((800 + rd(), 450 + rd()), 100), ((700 + rd(), 200 + rd()), 100), ((800 + rd(), 350 + rd()), 100), ((800 + rd(), 700 + rd()), 100)]

		# map = map + map1 + map2 + map3

		if ut.point_in_circle(robot_pos, waypoint_pos, 10):
			continue
		
		###   LOCAL PATH   ###
		start_time_cpt = time.perf_counter()
		
		safe_local_waypoint = find_next_local_waypoint(map, window_size, robot_pos, waypoint_pos, safe_range)

		end_time_cpt = time.perf_counter()

		if 1/(end_time_cpt-start_time_cpt) < 1000:
			print(f"Local path enlapse time : {end_time_cpt-start_time_cpt:.3e}s")
			print(f"Local path FPS : {1/(end_time_cpt-start_time_cpt):.1f} FPS")

		###   GLOBAL PATH   ###
		# start_time_cpt = time.perf_counter()
		
		# safe_local_waypoints = find_global_path(map, window_size, robot_pos, waypoint_pos, safe_range, window)

		# end_time_cpt = time.perf_counter()


		# if 1/(end_time_cpt-start_time_cpt) < 4000:
		# 	print(f"Global path enlapse time : {end_time_cpt-start_time_cpt:.3e}s")
		# 	print(f"Global path FPS : {1/(end_time_cpt-start_time_cpt):.1f} FPS")
		if not ut.point_in_circle(robot_pos, last_static_pos, 50): 
			last_static_pos = robot_pos
			look_for_path_through_known_map = False
			static_pos_counter = 0
		else:
			static_pos_counter += 1
		
		if static_pos_counter > 20:
			print("Robot is in a dead lock")
			look_for_path_through_known_map = True

		robot_pos = move_robot(robot_pos, safe_local_waypoint)
		total_dist += 10
		print(f"Safe path found, distance of the path : {total_dist:.2f} um")

		# DRAW THE SCENE
		update_display(window, map, robot_pos, waypoint_pos, safe_local_waypoint, None)
		text_shown = False
		next_it = False

	### DEINITIALIZE PYGAME
	pygame.quit()


if __name__ == '__main__':
	main()