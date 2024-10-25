# Create a map
import pygame
from pygame.locals import *
import time
from util import compute_colision
from robot import BeaconRobot
from map import Map

def write_fps(dt, window, window_size):
	font = pygame.font.Font('freesansbold.ttf', 16)
	if dt:
		text = font.render(f'{1/dt:.1f}', True, (255, 255, 255))

		textRect = text.get_rect()
	
		textRect.center = (window_size[0]*0.9, window_size[1]*0.1)
		window.blit(text, textRect)


def main():
	### Init pygame window
	pygame.init()

	window_size = (900, 600)
	map_size = (800, 500)
	map_offset = (50, 50)
	subdiv_number = (20, 20)
	window = pygame.display.set_mode(window_size)
	pygame.display.set_caption("Map")

	map = Map(map_size, map_offset, subdiv_number, 20, 20)
	# walls = [wall.get_rect() for wall in map.walls]

	beacon = BeaconRobot((200,400), 50, 1000, 50, -25, 100)
	beacon.equip_lidar(fov=360, freq=5, res=3.5, prec=5)
	beacon.equip_accmeter(prec=5)

	running = True
	t = time.time()
	t_old = t
	while running:
		key_pressed_is = pygame.key.get_pressed()
		# Handle events
		if key_pressed_is[K_ESCAPE]: 
			running = False
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
		
		# Draw background and map
		window.fill((150, 150, 150))
		# map.draw_map(window)

		# compute_colision(beacon, walls)

		dt = t - t_old
		rotation = 0
		direction = 0
		if key_pressed_is[K_LEFT]:
			rotation -= 1
		if key_pressed_is[K_RIGHT]: 
			rotation += 1
		if key_pressed_is[K_UP]: 
			direction += 1
		if key_pressed_is[K_DOWN]: 
			direction -= 1
		
		beacon.move(direction, dt)
		beacon.rotate(rotation, dt)

		env_scanned = beacon.scan_environment(t, map, window)
		beacon.compute_pos_calc(t)
		# beacon.draw_known_map(window)
		beacon.update_live_grid_map(None)
		if env_scanned:
			beacon.draw_live_grid_map(window)

		# Draw character
		beacon.draw(window)

		write_fps(dt, window, window_size)

		t_old = t
		t = time.time()
		
		pygame.display.update()
	pygame.quit()

if __name__ == '__main__':
	main()