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
		"""Draw the waypoint as a circle

		Args:
			window (surface): Surface on which to draw
		"""
		color = (0, 255, 100)
		pygame.draw.circle(window, color, waypoint_pos, 10)

def draw_sub_wps(window, valid_sub_wps, color):
	for sub_wp in valid_sub_wps:
		pygame.draw.circle(window, color, sub_wp, 10)


def draw_map(window, map):
	for c in map:
		pygame.draw.circle(window, (0, 0, 0), c[0], c[1], 3)

def update_display(window, map, robot_pos, waypoint_pos, valid_up_sub_wp, valid_down_sub_wp):
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
	# Draw the sub-waypoints
	draw_sub_wps(window, valid_up_sub_wp, (200, 0, 255))
	draw_sub_wps(window, valid_down_sub_wp, (0, 200, 255))
	# Draw the waypoint
	draw_waypoint(window, waypoint_pos)

	if len(valid_up_sub_wp):
		pygame.draw.line(window, (10, 10, 10), robot_pos, valid_up_sub_wp[0], 2)

		for i in range(len(valid_up_sub_wp) - 1):
			pygame.draw.line(window, (10, 10, 10), valid_up_sub_wp[i], valid_up_sub_wp[i+1], 2)

	
	# Update scene display
	pygame.display.update()

def need_line_split(map, p1, p2, line):
	safe_range = 25
	for c in map:
		ortho_point = ut.orthogonal_point(c[0], p1, p2, line)
		if ut.distance_tuple(c[0], ortho_point) < c[1] + safe_range:
			if ut.point_in_box_tuple(ortho_point, p1, p2):
				return True
	
	return False

def split_line(map, map_size, p1, p2, line):
	safe_range = 50

	a, b, _ = line	# ax + by = c
	if a:
		ortho_vec = ut.normalize_vec((a, b))
	else:
		ortho_vec = (0, ut.sign(b)) 
	
	mdp_safe = False
	mdp = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	
	while not mdp_safe:
		mdp = (mdp[0] + safe_range * ortho_vec[0], mdp[1] + safe_range * ortho_vec[1])

		if not ut.point_in_box_tuple(mdp, (0, 0), map_size):
			mdp = None
			break

		for c in map:
			if ut.distance_tuple(c[0], mdp) < c[1] + safe_range:
				mdp_safe = False
				break
			mdp_safe = True
	
	return mdp

		

def main():
	### INITIALISAZE PYGAME
	pygame.init()

	### TIME INIT
	real_time = time.time()
	t = 0
	dt = 2e-3

	### WINDOW INITIALISATION
	window_size = (1200, 700)
	# Initialize the window
	window = pygame.display.set_mode(window_size)
	# Set the title of the window
	pygame.display.set_caption("Test evitement")


	### WORLD INITIALISATION
	robot_pos = (200, 350)
	waypoint_pos = (1000, 350)
	# map = [((600, 250), 200), ((800, 450), 200)]*10
	map = [((400, 250), 100), ((800, 450), 100)]

	# State of the simulation
	running = True
	next_it = False
	text_shown = False

	# Simulation variables 
	sub_waypoints = [waypoint_pos]
	valid_up_sub_wp = [waypoint_pos]
	valid_down_sub_wp = []
	new_valid_up_sub_wp = []


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
		a = time.perf_counter()
		# while True:
		if len(valid_up_sub_wp):

				p1 = robot_pos
				p2 = valid_up_sub_wp[0]
				line = ut.compute_line_tuple(p1, p2)
				if need_line_split(map, p1, p2, line):
					new_valid_up_sub_wp = []
					mdp_up = split_line(map, window_size, p1, p2, line)
					if mdp_up:
						new_valid_up_sub_wp.append(mdp_up)
					else:
						print("Be careful, None detected !")

				
				# for i in range(len(valid_up_sub_wp) - 1):
				# 	p1 = valid_up_sub_wp[i]
				# 	p2 = valid_up_sub_wp[i+1]
				# 	new_valid_up_sub_wp.append(valid_up_sub_wp[i])
				# 	line = ut.compute_line_tuple(p1, p2)
				# 	if need_line_split(map, p1, p2, line):
				# 		mdp_up = split_line(map, window_size, p1, p2, line)
				# 		if mdp_up:
				# 			new_valid_up_sub_wp.append(mdp_up)
				# 		else:
				# 			print("Be careful, None detected !")

					new_valid_up_sub_wp.append(waypoint_pos)

				else:
					total_dist = ut.distance_tuple(robot_pos, valid_up_sub_wp[0])
					for i in range(len(valid_up_sub_wp) - 1):
						total_dist += ut.distance_tuple(valid_up_sub_wp[i], valid_up_sub_wp[i+1])

					print(f"Safe path found, distance of the path : {total_dist:.2f} um")
					robot_pos = new_valid_up_sub_wp.pop(0)
					new_valid_up_sub_wp = [waypoint_pos]
					
					
				valid_up_sub_wp = new_valid_up_sub_wp[:]
		b = time.perf_counter()

		print(f"Enlapse time : {b-a:.3e}s")
		print(f"FPS : {1/(b-a):.1f} FPS")


		# DRAW THE SCENE
		update_display(window, map, robot_pos, waypoint_pos, valid_up_sub_wp, valid_down_sub_wp)
		text_shown = False
		next_it = False

	### DEINITIALIZE PYGAME
	pygame.quit()


if __name__ == '__main__':
	main()