# Create a map
import pygame
from pygame.locals import *
import time
from util import compute_colision
from robot import BeaconRobot
from map import Map


def main():
	### Init pygame window
	pygame.init()

	window_size = (800, 600)
	window = pygame.display.set_mode(window_size)
	pygame.display.set_caption("Map")

	map = Map()
	walls = [wall.get_rect() for wall in map.walls]

	beacon = BeaconRobot((200,400), 50, 50)
	beacon.equip_lidar(fov=360, freq=5, res=3.5, prec=4)

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
		map.draw_map(window)

		compute_colision(beacon, walls)

		dt = t - t_old
		dir = [0, 0, 0, 0]
		if key_pressed_is[K_LEFT]:
			dir[0] = 1
		if key_pressed_is[K_RIGHT]: 
			dir[1] = 1
		if key_pressed_is[K_UP]: 
			dir[2] = 1
		if key_pressed_is[K_DOWN]: 
			dir[3] = 1
		
		beacon.move(dir, dt)

		# Draw character
		beacon.draw(window)
		beacon.scan_environment(t, walls)
		beacon.draw_known_map(window)


		t_old = t
		t = time.time()
		
		pygame.display.update()
	pygame.quit()

if __name__ == '__main__':
	main()