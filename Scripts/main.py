import pygame
from pygame.locals import *
 
import time
import numpy as np
np.random.seed(5)

from world import World


def write_fps(dt, window, window_size):
	"""Write the FPS on the top-right of the window

	Args:
		dt (float): Time between two frame
		window (surface): Window on which writting
		window_size (tuple): Size of the window
	"""
	font = pygame.font.Font('freesansbold.ttf', 20)
	if dt:
		text = font.render(f'FPS {1/dt:.1f}', True, (255, 255, 255))

		textRect = text.get_rect()
	
		textRect.center = (window_size[0]*0.9, window_size[1]*0.05)
		window.blit(text, textRect)

def write_time(time, window, window_size):
	"""Write the FPS on the top-right of the window

	Args:
		dt (float): Time between two frame
		window (surface): Window on which writting
		window_size (tuple): Size of the window
	"""
	font = pygame.font.Font('freesansbold.ttf', 20)
	formated_time = f"{time//3600:g} h, {(time - time//3600)//60:g} m, {time - (time//3600)*3600 - ((time - time//3600)//60)*60:.3f} s"
	text = font.render(formated_time, True, (255, 255, 255))

	textRect = text.get_rect()

	textRect.center = (window_size[0]*0.5, window_size[1]*0.05)
	window.blit(text, textRect)


def draw_links(world:World, window):
	for rs in world.linked_robot:
		r1, r2 = world.robots[rs[0]], world.robots[rs[1]]
		
		pygame.draw.line(window, (100, 50, 255), r1.pos.to_tuple(), r2.pos.to_tuple())
	

# def write_robot_info(dt, window, window_size, beacon:BeaconRobot):
# 	"""Write the robot info on the bottom of the window

# 	Args:
# 		dt (float): Time between two frame
# 		window (surface): Window on which writting
# 		window_size (tuple): Size of the window
# 		beacon (BeaconRobot): Robot class
# 	"""
# 	font = pygame.font.Font('freesansbold.ttf', 16)
# 	if dt:
# 		text1 = font.render(f"Calculated : [Acc {beacon.acc_calc:.1f}, Speed {beacon.speed_calc:.1f}, Pos {beacon.pos_calc}]; Real : [Acc {beacon.acc:.1f}, Speed {beacon.speed:.1f}, Pos {beacon.pos}]", True, (255, 255, 255))
# 		text2 = font.render(f"Calculated : [Ang acc {beacon.ang_acc_calc:.1f}, Ang speed {beacon.rot_speed_calc:.1f}, Angle {beacon.rot_calc:.1f}]; Real : [Ang acc {beacon.ang_acc:.1f}, Ang speed {beacon.rot_speed:.1f}, Angle {beacon.rot:.1f}]", True, (255, 255, 255))

# 		textRect1 = text1.get_rect()
# 		textRect2 = text2.get_rect()
	
# 		textRect1.center = (window_size[0]*0.5, window_size[1]*0.98)
# 		textRect2.center = (window_size[0]*0.5, window_size[1]*0.95)
# 		window.blit(text1, textRect1)
# 		window.blit(text2, textRect2)

def write_crashed_robot(window, window_size):
	font1 = pygame.font.Font('freesansbold.ttf', 54)
	font2 = pygame.font.Font('freesansbold.ttf', 32)
	
	text1 = font1.render("ROBOT CRASHED !!!", True, (255, 50, 100))
	text2 = font2.render("End of the simulation...", True, (255, 50, 100))
	
	textRect1 = text1.get_rect()
	textRect2 = text2.get_rect()
	
	textRect1.center = (window_size[0]*0.5, window_size[1]*0.5)
	textRect2.center = (window_size[0]*0.5, window_size[1]*0.6)
	
	window.blit(text1, textRect1)
	window.blit(text2, textRect2)

def update_display(window, window_size:float, world:World, time:float):
	"""Update the display

	Args:
		window (surface): Surface on which the scene is drawn
		window_size (tuple): Window size
		map (Map): Scene map
		robots (List(BeaconRobot)): List of robots
		dt (float): Display time step
	"""
	map = world.map
	robots = world.robots
	# Erase the display
	window.fill((150, 150, 150))

	map.draw_map(window)
	# map.draw_subdiv_points(window)

	# Draw communication links
	draw_links(world, window)
	
	for robot in robots:
		if robot.controller.mode:
			if robot.controller.local_waypoint is not None:
				pygame.draw.circle(window, (255, 0, 0), robot.controller.local_waypoint, robot.controller.waypoint_radius)
			if robot.controller.waypoint is not None:
				pygame.draw.circle(window, (255, 0, 0), robot.controller.waypoint, robot.controller.waypoint_radius)

		# Draw the robot
		robot.live_grid_map.draw(window)
		robot.draw(window)
	# Draw the FPS
	write_time(time, window, window_size)
	# write_robot_info(time, window, window_size, beacon)
	# if beacon_crashed:
	# 	write_crashed_robot(window, window_size)

	# Update scene display
	pygame.display.update()


def main():
	### INITIALISAZE PYGAME
	pygame.init()

	### WINDOW INITIALISATION
	window_size = (1200, 700)
	window = pygame.display.set_mode(window_size)
	pygame.display.set_caption("Map")
	
	### WORLD INIT
	main_world = World(window)


	# State of the simulation
	running = True

	# Display time
	real_time = time.time()
	t_display = real_time
	desired_fps = 25

	while running:
		real_time = time.time()
		t = main_world.time
		key_pressed_is = pygame.key.get_pressed()
		# Handle events
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
			# Handle if the key was pressed once
			if event.type == KEYDOWN:
				if event.key == K_m:
					toggle_draw_map = not toggle_draw_map
				if event.key == K_l:
					toggle_draw_live_map = not toggle_draw_live_map
				if event.key == K_ESCAPE:
					running = False
		if key_pressed_is[K_RIGHT]:
			desired_fps = min(500, desired_fps + 20*1/desired_fps)
		if key_pressed_is[K_LEFT]:
			desired_fps = max(1, desired_fps - 20*1/desired_fps)

		main_world.update()

		if main_world.map_explored:
			running = False

		# DRAW THE SCENE
		if real_time - t_display > 1/desired_fps:
			update_display(window, window_size, main_world, t)
			t_display = real_time

	print(f"Time to explore all the map : {t:.3f} s")
	print(f"Time to explore all the map : {t//3600:g} h, {(t - t//3600)//60:g} m, {t - (t//3600)*3600 - ((t - t//3600)//60)*60:.3f} s")
	
	### DEINITIALIZE PYGAME
	pygame.quit()


if __name__ == '__main__':
	main()