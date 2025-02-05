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

		main_world.update()
		main_window.update(main_world)

		if main_world.map_explored or main_world.end_simulation:
			running = False
	
	main_window.destroy()

	t = main_world.time
	print(f"Time to explore all the map : {t:.3f} s")
	print(f"Time to explore all the map : {t//3600:g} h, {(t - (t//3600)*3600)//60:g} m, {t - (t//3600)*3600 - ((t - (t//3600)*3600)//60)*60:.3f} s")


if __name__ == '__main__':
	main()