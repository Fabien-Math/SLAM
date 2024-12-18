import pygame
from pygame.locals import *
 
import time
import numpy as np
np.random.seed(5)

from robot import BeaconRobot
from map import Map


def write_fps(dt, window, window_size):
	"""Write the FPS on the top-right of the window

	Args:
		dt (float): Time between two frame
		window (surface): Window on which writting
		window_size (tuple): Size of the window
	"""
	font = pygame.font.Font('freesansbold.ttf', 16)
	if dt:
		text = font.render(f'FPS {1/dt:.1f}', True, (255, 255, 255))

		textRect = text.get_rect()
	
		textRect.center = (window_size[0]*0.9, window_size[1]*0.1)
		window.blit(text, textRect)


def write_robot_info(dt, window, window_size, beacon:BeaconRobot):
	"""Write the robot info on the bottom of the window

	Args:
		dt (float): Time between two frame
		window (surface): Window on which writting
		window_size (tuple): Size of the window
		beacon (BeaconRobot): Robot class
	"""
	font = pygame.font.Font('freesansbold.ttf', 16)
	if dt:
		text1 = font.render(f"Calculated : [Acc {beacon.acc_calc:.1f}, Speed {beacon.speed_calc:.1f}, Pos {beacon.pos_calc}]; Real : [Acc {beacon.acc:.1f}, Speed {beacon.speed:.1f}, Pos {beacon.pos}]", True, (255, 255, 255))
		text2 = font.render(f"Calculated : [Ang acc {beacon.ang_acc_calc:.1f}, Ang speed {beacon.rot_speed_calc:.1f}, Angle {beacon.rot_calc:.1f}]; Real : [Ang acc {beacon.ang_acc:.1f}, Ang speed {beacon.rot_speed:.1f}, Angle {beacon.rot:.1f}]", True, (255, 255, 255))

		textRect1 = text1.get_rect()
		textRect2 = text2.get_rect()
	
		textRect1.center = (window_size[0]*0.5, window_size[1]*0.98)
		textRect2.center = (window_size[0]*0.5, window_size[1]*0.95)
		window.blit(text1, textRect1)
		window.blit(text2, textRect2)

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

def update_display(window, window_size:float, map:Map, beacon:BeaconRobot, dt:float, toggle_draw_map:bool, toggle_draw_live_map:bool, beacon_crashed:bool):
	"""Update the display

	Args:
		window (surface): Surface on which the scene is drawn
		window_size (tuple): Window size
		map (Map): Scene map
		beacon (BeaconRobot): Robot
		dt (float): Display time step
		env_scanned (bool): Redraw the scene when the live grid map is updated
		toggle_draw_map (bool): Toggle to draw the map
	"""
	
	# Erase the display
	window.fill((150, 150, 150))

	if toggle_draw_live_map:
		beacon.live_grid_map.draw(window)
		beacon.draw_dot_map(window)
	if toggle_draw_map:
		map.draw_map(window)
	
	# map.draw_subdiv_points(window)
	

	if beacon.controller.mode:
		if beacon.controller.local_waypoint is not None:
			pygame.draw.circle(window, (255, 0, 0), beacon.controller.local_waypoint, beacon.controller.waypoint_radius)
		if beacon.controller.waypoint is not None:
			pygame.draw.circle(window, (255, 0, 0), beacon.controller.waypoint, beacon.controller.waypoint_radius)

	# Draw the robot
	beacon.draw(window)
	# Draw the FPS
	write_fps(dt, window, window_size)
	write_robot_info(dt, window, window_size, beacon)
	if beacon_crashed:
		write_crashed_robot(window, window_size)

	# Update scene display
	pygame.display.update()


def main():
	### INITIALISAZE PYGAME
	pygame.init()

	### TIME INIT
	real_time = time.time()
	t = 0
	dt = 1e-3

	### WINDOW INITIALISATION
	window_size = (1200, 700)
	# Initialize the window
	window = pygame.display.set_mode(window_size)
	# Set the title of the window
	pygame.display.set_caption("Map")

	# MAP INITIALISATION
	map_size = (1100, 600)
	map_offset = (50, 50)
	# Number of subdivisions in the map, used to list the lines
	subdiv_number = (25, 25)
	# Initilize the map
	map = Map(map_size, map_offset, subdiv_number, 40, int(np.sin(np.pi/3) * 40))


	### ROBOT INITIALISATION
	beacon = BeaconRobot((500, 300), 50, 1000, 100, 25, 150)
	# beacon = BeaconRobot((300, 400), 50, 1000, 100, 25, 150)
	# Equip sensors
	beacon.equip_lidar(fov=360, freq=2, res=3.5, prec=(0.05, 0.02), max_dist=100)
	beacon.equip_accmeter(precision=(0.0005, 0.00002), time=t)
	beacon.equip_controller(check_safe_path_frequency=5, mode=1)

	# State of the simulation
	running = True
	# Toggle to draw the map
	toggle_draw_map = True
	toggle_draw_live_map = True

	beacon_crashed = False
	
	# Display time
	t_display = real_time
	desired_fps = 20
	while running:
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
				if event.key == K_a:
					beacon.controller.mode = 1 * (beacon.controller.mode == 0) + 0 * (beacon.controller.mode != 0)
				if event.key == K_ESCAPE: 
					running = False
		if key_pressed_is[K_RIGHT]:
			desired_fps = min(500, desired_fps + 20*1/desired_fps)
		if key_pressed_is[K_LEFT]:
			desired_fps = max(1, desired_fps - 20*1/desired_fps)
		

		### SIMULATION
		beacon.compute_pos_calc(t)
		beacon.scan_environment(t, map, window)
		beacon.compute_robot_collision(map, window)
		beacon.live_grid_map.update_robot_path()

		if beacon.controller.mode:
			# beacon.controller.check_safe_path(t)
			beacon.controller.move_to_waypoint(dt, window)
		else:

			# Direction for rotation and direction
			rotation = 0
			direction = 0
			if key_pressed_is[K_q]:
				rotation -= 1
			if key_pressed_is[K_d]: 
				rotation += 1
			if key_pressed_is[K_z]: 
				direction += 1
			if key_pressed_is[K_s]: 
				direction -= 1

			# Robot movement
			beacon.move(dt, direction)
			beacon.rotate(dt, rotation)
		
		# TIME
		real_time = time.time()
		t += dt
		
		# TIMER
		if beacon.crashed_in_wall:
			t_crashed = t
			beacon_crashed = True

		if beacon.controller.mode == 100:
			beacon.live_grid_map.save_map_to_image()
			running = False
		
		if beacon_crashed:
			if t > t_crashed + 2:
				...
				# running = False

		# DRAW THE SCENE

		if real_time - t_display > 1/desired_fps:
			update_display(window, window_size, map, beacon, real_time - t_display, toggle_draw_map, toggle_draw_live_map, beacon_crashed)
			t_display = real_time

	print(f"Time to explore all the map : {t:.3f} s")
	print(f"Time to explore all the map : {t//3600:g} h, {(t - t//3600)//60:g} m, {t - (t//3600)*3600 - ((t - t//3600)//60)*60:.3f} s")
	### DEINITIALIZE PYGAME
	pygame.quit()


if __name__ == '__main__':
	main()