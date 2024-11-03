# Create a map
import pygame
from pygame.locals import *
import time
from robot import BeaconRobot
from map import Map
import numpy as np

np.random.seed(5)

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
		text = font.render(f"Calculated : [Acc {beacon.acc_calc:.1f}, Speed {beacon.speed_calc:.1f}, Pos {beacon.pos_calc}]; Real : [Acc {beacon.acc:.1f}, Speed {beacon.speed:.1f}, Pos {beacon.pos}]", True, (255, 255, 255))

		textRect = text.get_rect()
	
		textRect.center = (window_size[0]*0.5, window_size[1]*0.95)
		window.blit(text, textRect)


def update_display(window, window_size:float, map:Map, beacon:BeaconRobot, dt:float, toggle_draw_map:bool, toggle_draw_live_map:bool):
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
		beacon.draw_live_grid_map(window)
	if toggle_draw_map:
		map.draw_map(window)

	beacon.controller.draw_voronoi_diagram(window)

	# Draw the robot
	beacon.draw(window)
	# Draw the FPS
	write_fps(dt, window, window_size)
	write_robot_info(dt, window, window_size, beacon)

	# Update scene display
	pygame.display.update()


def main():
	### INITIALISAZE PYGAME
	pygame.init()

	### WINDOW INITIALISATION
	window_size = (1300, 700)
	# Initialize the window
	window = pygame.display.set_mode(window_size)
	# Set the title of the window
	pygame.display.set_caption("Map")

	# MAP INITIALISATION
	map_size = (1200, 600)
	map_offset = (50, 50)
	# Number of subdivisions in the map, used to list the lines
	subdiv_number = (20, 20)
	# Initilize the map
	map = Map(map_size, map_offset, subdiv_number, 25, 25)


	### ROBOT INITIALISATION
	beacon = BeaconRobot((350, 275), 50, 1000, 50, -25, 100)
	# Equip sensors
	beacon.equip_lidar(fov=360, freq=5, res=3.5, prec=5, max_dist=200)
	beacon.equip_accmeter(acc_prec=5, ang_acc_prec=0.2)

	# State of the simulation
	running = True
	# Toggle to draw the map
	toggle_draw_map = True
	toggle_draw_live_map = True

	# Desired FPS
	desired_fps = 60
	

	t = time.time()
	t_old = t
	
	# Display time
	t_display = t
	while running:
		# Time elapsed since the previous step
		dt = t - t_old


		key_pressed_is = pygame.key.get_pressed()
		# Handle events
		if key_pressed_is[K_ESCAPE]: 
			running = False
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
		if key_pressed_is[K_m]:
			toggle_draw_map = not toggle_draw_map
		if key_pressed_is[K_l]:
			toggle_draw_live_map = not toggle_draw_live_map
		if key_pressed_is[K_UP]:
			desired_fps = min(200, desired_fps + 20*dt)
		if key_pressed_is[K_DOWN]:
			desired_fps = max(1, desired_fps - 20*dt)

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
		beacon.move(direction, dt)
		beacon.rotate(rotation, dt)

		### SIMULATION
		beacon.scan_environment(t, map, window)
		beacon.compute_pos_calc(t)
		beacon.update_live_grid_map(None)
		# compute_colision(beacon, walls)
		
		# DRAW THE SCENE
		if t - t_display > 1/desired_fps:
			update_display(window, window_size, map, beacon, t - t_display, toggle_draw_map, toggle_draw_live_map)
			t_display = t


		t_old = t
		t = time.time()

	### DEINITIALIZE PYGAME
	pygame.quit()


if __name__ == '__main__':
	main()