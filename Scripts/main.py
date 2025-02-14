import pygame
from pygame.locals import *
 
import numpy as np
np.random.seed(5)

from display import Window
from world import World


def main():
	### INITIALISAZE WINDOW
	main_window = Window(window_size = (1200, 700), window_title = "Robot simulator")
	### WORLD INIT
	main_world = World(main_window.window)
	
	draw_map = True
	draw_wps = True
	draw_lgm = True
	draw_rnb = True
	draw_lks = True
	draw_rts = True
	pause = False

	# State of the simulation
	running = True
	while running:		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
			# Handle if the key was pressed once
			if event.type == KEYDOWN:
				if event.key == K_ESCAPE:
					running = False
				if event.key == K_RIGHT:
					main_world.active_robot = (main_world.active_robot + 1)%len(main_world.robots)
				if event.key == K_LEFT:
					main_world.active_robot = (main_world.active_robot - 1)%len(main_world.robots)
				if event.key == K_m:
					draw_map = not draw_map
				if event.key == K_l:
					draw_lgm = not draw_lgm
				if event.key == K_w:
					draw_wps = not draw_wps
				if event.key == K_n:
					draw_rnb = not draw_rnb
				if event.key == K_k:
					draw_lks = not draw_lks
				if event.key == K_r:
					draw_rts = not draw_rts
				if event.key == K_SPACE:
					pause = not pause

		if not pause:
			main_world.update()
		main_window.update(main_world, draw_rts, draw_lgm, draw_wps, draw_lks, draw_rnb, draw_map)

		if main_world.map_explored or main_world.end_simulation:
			running = False
	
	main_window.destroy()

	t = main_world.time
	print(f"Time to explore all the map : {t:.3f} s")
	print(f"Time to explore all the map : {t//3600:g} h, {(t - (t//3600)*3600)//60:g} m, {t - (t//3600)*3600 - ((t - (t//3600)*3600)//60)*60:.3f} s")


if __name__ == '__main__':
	main()